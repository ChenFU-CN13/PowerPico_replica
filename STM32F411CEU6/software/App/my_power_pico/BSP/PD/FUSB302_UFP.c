
/**
 * FUSB302_UFP.c
 *
 *  Updated on: Jan 4, 2021
 *      Author: Ryan Ma
 *
 * Minimalist USB PD implement with only UFP(device) functionality
 * Requires only stdint.h and string.h
 * No use of bit-field for better cross-platform compatibility
 *
 * FUSB302 can support PD3.0 with limitations and workarounds
 * - Do not have enough FIFO for unchunked message, use chunked message instead
 * - VBUS sense low threshold at 4V, disable vbus_sense if request PPS below 4V
 *
 */

/*
 * 中文阅读说明：
 * 本文件是 FUSB302B 的“寄存器/FIFO 驱动层”，不负责决定要请求几伏。
 * - FUSB302_state_unattached()：检测 VBUS 和 CC1/CC2，确认插头方向。
 * - FUSB302_state_attached()：读取中断、判断断开、取出 RX FIFO 数据。
 * - FUSB302_tx_sop()：把 PD 消息头和 Data Object 包装成 FUSB302 TX FIFO token。
 * - FUSB302_alert()：按当前 attached/unattached 状态分派处理函数。
 *
 * 寄存器位定义已按 FUSB302B 官方 datasheet 的 Register Definitions 表核对。
 * 复刻时建议先观察 I2C 读写和 INT_N，再进入上层 PDO/APDO 策略调试。
 */

#include <string.h>
#include "FUSB302_UFP.h"

/* Switches0 : 02h  —— CC 引脚开关与测量通路选择寄存器
 *  bit7  PU_EN2      : CC2 引脚内部上拉使能（作为 Source/DFP 时需要）
 *  bit6  PU_EN1      : CC1 引脚内部上拉使能
 *  bit5  VCONN_CC2   : 通过 CC2 向对端提供 VCONN(5V) 电源
 *  bit4  VCONN_CC1   : 通过 CC1 向对端提供 VCONN
 *  bit3  MEAS_CC2    : 选择 CC2 作为 ADC 测量输入（测电压/电流判定方向）
 *  bit2  MEAS_CC1    : 选择 CC1 作为 ADC 测量输入
 *  bit1  PDWN2       : CC2 引脚内部下拉使能（作为 Sink/UFP 时需要）
 *  bit0  PDWN1       : CC1 引脚内部下拉使能
 */
#define PU_EN2          (0x01 << 7)
#define PU_EN1          (0x01 << 6)
#define VCONN_CC2       (0x01 << 5)
#define VCONN_CC1       (0x01 << 4)
#define MEAS_CC2        (0x01 << 3)
#define MEAS_CC1        (0x01 << 2)
#define PDWN2           (0x01 << 1)
#define PDWN1           (0x01 << 0)

/* Switches1 : 03h  —— 收发方向与 PD 版本配置寄存器
 *  bit7  POWERROLE   : 电源角色(0=Sink, 1=Source)，用于 GoodCRC 响应头
 *  bit6~5 SPECREV1/0 : 协议版本，00=PD1.0, 01=PD2.0, 10=PD3.0
 *  bit4  DATAROLE    : 数据角色(0=UFP, 1=DFP)
 *  bit2  AUTO_CRC    : 自动给收到的有效报文回 GoodCRC（必须开启）
 *  bit1  TXCC2       : 选择 CC2 作为发送通道
 *  bit0  TXCC1       : 选择 CC1 作为发送通道（与 TXCC2 互斥，按接入方向选其一）
 */
#define POWERROLE       (0x01 << 7)
#define SPECREV1        (0x01 << 6)
#define SPECREV0        (0x01 << 5)
#define DATAROLE        (0x01 << 4)
#define AUTO_CRC        (0x01 << 2)
#define TXCC2           (0x01 << 1)
#define TXCC1           (0x01 << 0)

/* Measure : 04h  —— 测量块选择寄存器
 *  bit6  MEAS_VBUS   : 选择 VBUS 作为 ADC 测量输入（用于判断 VBUS 是否达到目标电压）
 */
#define MEAS_VBUS       (0x01 << 6)

/* Control0 : 06h  —— 发送控制与主机电流通告寄存器
 *  bit6  TX_FLUSH     : 清空 TX FIFO（写 1 触发）
 *  bit5  INT_MASK     : 中断屏蔽总开关（1=屏蔽所有中断，0=正常）
 *  bit4~3 HOST_CUR_*  : 作为 Source 时通告的最大供电电流档位
 *                       01=Default(USB默认), 10=1.5A, 11=3.0A
 *  bit1  AUTO_PRE     : 发送前自动加 preamble（建议开启）
 *  bit0  TX_START     : 启动一次发送（TX FIFO 填好后置 1 触发）
 */
#define TX_FLUSH        (0x01 << 6)
#define INT_MASK        (0x01 << 5)
#define HOST_CUR_MASK   (0x03 << 2)
#define HOST_CUR_3A0    (0x03 << 2)
#define HOST_CUR_1A5    (0x02 << 2)
#define HOST_CUR_USB    (0x01 << 2)
#define AUTO_PRE        (0x01 << 1)
#define TX_START        (0x01 << 0)

/* Control1 : 07h  —— 接收 SOP 类型使能寄存器
 *  bit6  ENSOP2DB    : 使能接收 SOP2' Debug 报文
 *  bit5  ENSOP1DB    : 使能接收 SOP1' Debug 报文
 *  bit4  BIST_MODE2  : BIST 测试模式2
 *  bit2  RX_FLUSH    : 清空 RX FIFO（写 1 触发）
 *  bit1  ENSOP2      : 使能接收 SOP2 报文（Cable 标记）
 *  bit0  ENSOP1      : 使能接收 SOP1 报文（扩展标记）
 *  注：SOP 报文默认总是使能的，无需位控制
 */
#define ENSOP2DB        (0x01 << 6)
#define ENSOP1DB        (0x01 << 5)
#define BIST_MODE2      (0x01 << 4)
#define RX_FLUSH        (0x01 << 2)
#define ENSOP2          (0x01 << 1)
#define ENSOP1          (0x01 << 0)

/* Control2 : 08h  —— 模式与唤醒控制寄存器
 *  bit3  WAKE_EN     : 唤醒使能（检测到 CC 活动时唤醒器件）
 *  bit2~1 MODE_*     : 工作模式，01=DRP(双角色), 10=UFP(Sink), 11=DFP(Source)
 *  bit0  TOGGLE      : 启动角色切换探测（DRP 模式下使用）
 */
#define WAKE_EN         (0x01 << 3)
#define MODE_MASK       (0x03 << 1)
#define MODE_DFP        (0x03 << 1)
#define MODE_UFP        (0x02 << 1)
#define MODE_DRP        (0x01 << 1)
#define TOGGLE          (0x01 << 0)

/* Control3 : 09h  —— 复位与自动重发控制寄存器
 *  bit6  SEND_HARDRESET : 发送硬复位（写 1 触发，发送 Hard Reset 报文）
 *  bit5  BIST_TMODE     : BIST 测试模式（仅 FUSB302B）
 *  bit4  AUTO_HARDRESET : 收到无法识别报文时自动发 Hard Reset
 *  bit3  AUTO_SOFTRESET : 协议层错误时自动发 Soft Reset
 *  bit2~1 N_RETRIES(n)  : 发送失败后的最大重试次数（0~3 次）
 *  bit0  AUTO_RETRY     : 发送失败（未收到 GoodCRC）时自动重发
 */
#define SEND_HARDRESET  (0x01 << 6)
#define BIST_TMODE      (0x01 << 5)     /* on FUSB302B only */
#define AUTO_HARDRESET  (0x01 << 4)
#define AUTO_SOFTRESET  (0x01 << 3)
#define N_RETRIES_MASK  (0x03 << 1)
#define N_RETRIES(n)    ((n)  << 1)
#define AUTO_RETRY      (0x01 << 0)

/* Mask : 0Ah  —— 中断屏蔽寄存器 A（对应 Interrupt 0x42 的各个位）
 *  置 1 = 屏蔽该中断源（不产生 INT 引脚输出）；置 0 = 允许中断
 *  bit7  M_VBUSOK   : VBUS 状态变化中断屏蔽
 *  bit6  M_ACTIVITY : CC 线上有活动中断屏蔽
 *  bit5  M_COMP_CHNG: 比较器结果变化中断屏蔽
 *  bit4  M_CRC_CHK  : CRC 校验完成中断屏蔽
 *  bit3  M_ALERT    : 报警中断屏蔽
 *  bit2  M_WAKE     : 唤醒中断屏蔽
 *  bit1  M_COLLISION: 总线冲突中断屏蔽
 *  bit0  M_BC_LVL   : BC 电平变化中断屏蔽
 */
#define M_VBUSOK        (0x01 << 7)
#define M_ACTIVITY      (0x01 << 6)
#define M_COMP_CHNG     (0x01 << 5)
#define M_CRC_CHK       (0x01 << 4)
#define M_ALERT         (0x01 << 3)
#define M_WAKE          (0x01 << 2)
#define M_COLLISION     (0x01 << 1)
#define M_BC_LVL        (0x01 << 0)

/* Power : 0Bh  —— 电源管理寄存器（各模块上电控制）
 *  bit3  PWR_INT_OSC  : 内部振荡器上电
 *  bit2  PWR_MEASURE  : 测量模块上电
 *  bit1  PWR_RECEIVER : 接收模块上电（同时为测量块提供电流基准）
 *  bit0  PWR_BANDGAP  : 带隙基准与唤醒电路上电
 */
#define PWR_INT_OSC     (0x01 << 3) 	/* Enable internal oscillator */
#define PWR_MEASURE     (0x01 << 2) 	/* Measure block powered */
#define PWR_RECEIVER    (0x01 << 1) 	/* Receiver powered and current reference for Measure block */
#define PWR_BANDGAP     (0x01 << 0) 	/* Bandgap and wake circuitry */

/* Reset : 0Ch  —— 复位控制寄存器
 *  bit1  PD_RESET  : PD 逻辑复位（保留寄存器配置，仅复位 PD 协议状态机）
 *  bit0  SW_RESET  : 软件全复位（复位所有寄存器到默认值）
 */
#define PD_RESET        (0x01 << 1)
#define SW_RES          (0x01 << 0)

/* Maska : 0Eh  —— 中断屏蔽寄存器 A'（对应 Interrupta 0x3E 的各个位）
 *  bit7  M_OCP_TEMP  : 过流/过温中断屏蔽
 *  bit6  M_TOGDONE   : 角色切换(Toggle)完成中断屏蔽
 *  bit5  M_SOFTFAIL  : 软复位失败中断屏蔽
 *  bit4  M_RETRYFAIL : 重发失败中断屏蔽
 *  bit3  M_HARDSENT  : 硬复位已发送中断屏蔽
 *  bit2  M_TXSENT    : 报文发送完成中断屏蔽
 *  bit1  M_SOFTRST   : 收到软复位中断屏蔽
 *  bit0  M_HARDRST   : 收到硬复位中断屏蔽
 */
#define M_OCP_TEMP      (0x01 << 7)
#define M_TOGDONE       (0x01 << 6)
#define M_SOFTFAIL      (0x01 << 5)
#define M_RETRYFAIL     (0x01 << 4)
#define M_HARDSENT      (0x01 << 3)
#define M_TXSENT        (0x01 << 2)
#define M_SOFTRST       (0x01 << 1)
#define M_HARDRST       (0x01 << 0)

/* Maskb : 0Fh  —— 中断屏蔽寄存器 B（对应 Interruptb 0x3F）
 *  bit0  M_GCRCSENT  : GoodCRC 已发送中断屏蔽
 */
#define M_GCRCSENT      (0x01 << 0)

/* Status0a : 3Ch  —— 状态寄存器 A'
 *  bit5  SOFTFAIL    : 软复位失败标志
 *  bit4  RETRYFAIL   : 重发超限失败标志
 *  bit2  POWER3_2    : 电源电压高于 3.2V 标志
 *  bit1  SOFTRST     : 收到软复位报文标志
 *  bit0  HARDRST     : 收到硬复位报文标志
 */
#define SOFTFAIL        (0x01 << 5)
#define RETRYFAIL       (0x01 << 4)
#define POWER3_2        (0x01 << 2)
#define SOFTRST         (0x01 << 1)
#define HARDRST         (0x01 << 0)

/* Status1a : 3Dh  —— 状态寄存器 B'（角色切换状态 + 接收 SOP 类型）
 *  bit5~3 TOGSS_*    : Toggle 状态机当前状态
 *                     000=Running, 001=SRC1(CC1做Source), 010=SRC2(CC2做Source),
 *                     101=SNK1(CC1做Sink), 110=SNK2(CC2做Sink), 111=Audio
 *  bit2  RXSOP2DB    : 收到 SOP2' Debug 报文标志
 *  bit1  RXSOP1DB    : 收到 SOP1' Debug 报文标志
 *  bit0  RXSOP       : 收到 SOP 报文标志（普通 PD 报文）
 */
#define TOGSS_MASK      (0x07 << 3)
#define TOGSS_RUNNING   (0x00 << 3)
#define TOGSS_SRC1      (0x01 << 3)
#define TOGSS_SRC2      (0x02 << 3)
#define TOGSS_SNK1      (0x05 << 3)
#define TOGSS_SNK2      (0x06 << 3)
#define TOGSS_AUDIOA    (0x07 << 3)
#define RXSOP2DB        (0x01 << 2)
#define RXSOP1DB        (0x01 << 1)
#define RXSOP           (0x01 << 0)

/* Interrupta : 3Eh  —— 中断状态寄存器 A'（读清，对应 Maska 屏蔽）
 *  bit7  I_OCP_TEMP  : 过流/过温中断
 *  bit6  I_TOGDONE   : Toggle 完成中断
 *  bit5  I_SOFTFAIL  : 软复位失败中断
 *  bit4  I_RETRYFAIL : 重发失败中断
 *  bit3  I_HARDSENT  : 硬复位已发送中断
 *  bit2  I_TXSENT    : 报文发送完成中断
 *  bit1  I_SOFTRST   : 收到软复位中断
 *  bit0  I_HARDRST   : 收到硬复位中断
 */
#define I_OCP_TEMP      (0x01 << 7)
#define I_TOGDONE       (0x01 << 6)
#define I_SOFTFAIL      (0x01 << 5)
#define I_RETRYFAIL     (0x01 << 4)
#define I_HARDSENT      (0x01 << 3)
#define I_TXSENT        (0x01 << 2)
#define I_SOFTRST       (0x01 << 1)
#define I_HARDRST       (0x01 << 0)

/* Interruptb : 3Fh  —— 中断状态寄存器 B（读清，对应 Maskb 屏蔽）
 *  bit0  I_GCRCSENT  : GoodCRC 报文已发送中断
 */
#define I_GCRCSENT      (0x01 << 0)

/* Status0 : 40h  —— 主状态寄存器（VBUS/活动/BC 电平等）
 *  bit7  VBUSOK      : VBUS 电压在有效范围内
 *  bit6  ACTIVITY    : CC 线上检测到活动
 *  bit5  COMP        : 测量比较器输出（与 BC_LVL 配合判定 CC 电压区间）
 *  bit4  CRC_CHK     : CRC 校验完成
 *  bit3  ALERT       : 报警标志
 *  bit2  WAKE        : 唤醒事件
 *  bit1~0 BC_LVL_*   : CC 引脚电压区间（用于判断对端通告的电流能力）
 *                      00=<200mV, 01=200~660mV, 10=660~1230mV, 11=>1230mV
 */
#define VBUSOK          (0x01 << 7)
#define ACTIVITY        (0x01 << 6)
#define COMP            (0x01 << 5)
#define CRC_CHK         (0x01 << 4)
#define ALERT           (0x01 << 3)
#define WAKE            (0x01 << 2)
#define BC_LVL_MASK     (0x03 << 0)
#define BC_LVL_LT200    (0x00 << 0)
#define BC_LVL_200_660  (0x01 << 0)
#define BC_LVL_660_1230 (0x02 << 0)
#define BC_LVL_GT1230   (0x03 << 0)

/* Status1 : 41h  —— FIFO 与异常状态寄存器
 *  bit7  RXSOP2      : 收到 SOP2 报文
 *  bit6  RXSOP1      : 收到 SOP1 报文
 *  bit5  RX_EMPTY    : RX FIFO 空
 *  bit4  RX_FULL     : RX FIFO 满
 *  bit3  TX_EMPTY    : TX FIFO 空
 *  bit2  TX_FULL     : TX FIFO 满
 *  bit1  OVRTEMP     : 过温标志
 *  bit0  OCP         : 过流标志
 */
#define RXSOP2          (0x01 << 7)
#define RXSOP1          (0x01 << 6)
#define RX_EMPTY        (0x01 << 5)
#define RX_FULL         (0x01 << 4)
#define TX_EMPTY        (0x01 << 3)
#define TX_FULL         (0x01 << 2)
#define OVRTEMP         (0x01 << 1)
#define OCP             (0x01 << 0)

/* Interrupt : 42h  —— 主中断状态寄存器（读清，对应 Mask 屏蔽）
 *  bit7  I_VBUSOK    : VBUS 状态变化中断
 *  bit6  I_ACTIVITY  : CC 活动中断
 *  bit5  I_COMP_CHNG : 比较器结果变化中断
 *  bit4  I_CRC_CHK   : CRC 校验完成中断
 *  bit3  I_ALERT     : 报警中断
 *  bit2  I_WAKE      : 唤醒中断
 *  bit1  I_COLLISION : 总线冲突中断
 *  bit0  I_BC_LVL    : BC 电平变化中断
 */
#define I_VBUSOK        (0x01 << 7)
#define I_ACTIVITY      (0x01 << 6)
#define I_COMP_CHNG     (0x01 << 5)
#define I_CRC_CHK       (0x01 << 4)
#define I_ALERT         (0x01 << 3)
#define I_WAKE          (0x01 << 2)
#define I_COLLISION     (0x01 << 1)
#define I_BC_LVL        (0x01 << 0)

/* ===== FUSB302 寄存器 I2C 地址映射表 =====
 *  控制类寄存器（地址 0x01~0x0F）：可读写，映射到 dev->reg_control[] 数组
 *  状态/中断类寄存器（地址 0x3C~0x42）：只读，映射到 dev->reg_status[] 数组
 *  ADDRESS_FIFOS(0x43)：TX/RX FIFO 数据口，通过它收发 PD 报文 token
 */
#define ADDRESS_DEVICE_ID   0x01   // 器件 ID 寄存器
#define ADDRESS_SWITCHES0   0x02   // CC 开关与测量选择
#define ADDRESS_SWITCHES1   0x03   // 收发方向与版本
#define ADDRESS_MEASURE     0x04   // 测量输入选择
#define ADDRESS_SLICE       0x05   // 测量比较器阈值
#define ADDRESS_CONTROL0    0x06   // 发送控制
#define ADDRESS_CONTROL1    0x07   // 接收 SOP 使能
#define ADDRESS_CONTROL2    0x08   // 模式与唤醒
#define ADDRESS_CONTROL3    0x09   // 复位与重发
#define ADDRESS_MASK        0x0A   // 中断屏蔽
#define ADDRESS_POWER       0x0B   // 电源管理
#define ADDRESS_RESET       0x0C   // 复位控制
#define ADDRESS_MASKA       0x0E   // 中断屏蔽 A'
#define ADDRESS_MASKB       0x0F   // 中断屏蔽 B
#define ADDRESS_STATUS0A    0x3C   // 状态 A'
#define ADDRESS_STATUS1A    0x3D   // 状态 B'
#define ADDRESS_INTERRUPTA  0x3E   // 中断状态 A'
#define ADDRESS_INTERRUPTB  0x3F   // 中断状态 B
#define ADDRESS_STATUS0     0x40   // 主状态
#define ADDRESS_STATUS1     0x41   // FIFO/异常状态
#define ADDRESS_INTERRUPT   0x42   // 主中断状态
#define ADDRESS_FIFOS       0x43   // FIFO 数据口

/* ===== 寄存器访问宏（数组索引映射） =====
 *  设计技巧：用 "寄存器地址 - 基地址" 作为数组下标，
 *  把离散的 I2C 寄存器地址映射成连续的数组元素，
 *  这样可以用 REG_xxx 直接读写 dev->reg_control[]/reg_status[] 中的对应项。
 *  控制类基址 = ADDRESS_DEVICE_ID(0x01)
 *  状态类基址 = ADDRESS_STATUS0A(0x3C)
 */
#define REG_DEVICE_ID       dev->reg_control[ADDRESS_DEVICE_ID  - ADDRESS_DEVICE_ID]
#define REG_SWITCHES0       dev->reg_control[ADDRESS_SWITCHES0  - ADDRESS_DEVICE_ID]
#define REG_SWITCHES1       dev->reg_control[ADDRESS_SWITCHES1  - ADDRESS_DEVICE_ID]
#define REG_MEASURE         dev->reg_control[ADDRESS_MEASURE    - ADDRESS_DEVICE_ID]
#define REG_SLICE           dev->reg_control[ADDRESS_SLICE      - ADDRESS_DEVICE_ID]
#define REG_CONTROL0        dev->reg_control[ADDRESS_CONTROL0   - ADDRESS_DEVICE_ID]
#define REG_CONTROL1        dev->reg_control[ADDRESS_CONTROL1   - ADDRESS_DEVICE_ID]
#define REG_CONTROL2        dev->reg_control[ADDRESS_CONTROL2   - ADDRESS_DEVICE_ID]
#define REG_CONTROL3        dev->reg_control[ADDRESS_CONTROL3   - ADDRESS_DEVICE_ID]
#define REG_MASK            dev->reg_control[ADDRESS_MASK       - ADDRESS_DEVICE_ID]
#define REG_POWER           dev->reg_control[ADDRESS_POWER      - ADDRESS_DEVICE_ID]
#define REG_RESET           dev->reg_control[ADDRESS_RESET      - ADDRESS_DEVICE_ID]
#define REG_MASKA           dev->reg_control[ADDRESS_MASKA      - ADDRESS_DEVICE_ID]
#define REG_MASKB           dev->reg_control[ADDRESS_MASKB      - ADDRESS_DEVICE_ID]
#define REG_STATUS0A        dev->reg_status[ADDRESS_STATUS0A    - ADDRESS_STATUS0A]
#define REG_STATUS1A        dev->reg_status[ADDRESS_STATUS1A    - ADDRESS_STATUS0A]
#define REG_INTERRUPTA      dev->reg_status[ADDRESS_INTERRUPTA  - ADDRESS_STATUS0A]
#define REG_INTERRUPTB      dev->reg_status[ADDRESS_INTERRUPTB  - ADDRESS_STATUS0A]
#define REG_STATUS0         dev->reg_status[ADDRESS_STATUS0     - ADDRESS_STATUS0A]
#define REG_STATUS1         dev->reg_status[ADDRESS_STATUS1     - ADDRESS_STATUS0A]
#define REG_INTERRUPT       dev->reg_status[ADDRESS_INTERRUPT   - ADDRESS_STATUS0A]

/* FUSB302 TX FIFO 发送令牌（Token）枚举
 *  往 TX FIFO 写入报文时，需要在数据前后插入这些 token 来界定报文结构：
 *  TXON  -> SOP1/SOP2/SOP3 -> PACKSYM(后续是报文数据) -> JAM_CRC -> EOP -> TXOFF
 */
enum FUSB302_transmit_data_tokens_t {
    TX_TOKEN_TXON    = 0xA1,  // 开启发送（使能 CC 发送驱动器）
    TX_TOKEN_SOP1    = 0x12,  // SOP1 报文起始定界符
    TX_TOKEN_SOP2    = 0x13,  // SOP2 报文起始定界符
    TX_TOKEN_SOP3    = 0x1B,  // SOP3 报文起始定界符
    TX_TOKEN_RESET1  = 0x15,  // Hard Reset 序列 1
    TX_TOKEN_RESET2  = 0x16,  // Hard Reset 序列 2
    TX_TOKEN_PACKSYM = 0x80,  // 后续数据为报文符号（packed symbols）
    TX_TOKEN_JAM_CRC = 0xFF,  // 让硬件自动追加 CRC32
    TX_TOKEN_EOP     = 0x14,  // 报文结束定界符
    TX_TOKEN_TXOFF   = 0xFE,  // 关闭发送（关闭 CC 发送驱动器）
};

/* FUSB302 连接状态枚举 */
enum FUSB302_state_t {
    FUSB302_STATE_UNATTACHED = 0,  // 未连接（未检测到对端 CC 上拉/下拉）
    FUSB302_STATE_ATTACHED         // 已连接（识别到对端角色）
};

#define FUSB302_ERR_MSG(s)  s

#define REG_READ(addr, data, count) do { \
    if (reg_read(dev, addr, data, count) != FUSB302_SUCCESS) { return FUSB302_ERR_READ_DEVICE; } \
} while(0)

#define REG_WRITE(addr, data, count) do { \
    if (reg_write(dev, addr, data, count) != FUSB302_SUCCESS) { return FUSB302_ERR_WRITE_DEVICE; } \
} while(0)

static inline FUSB302_ret_t reg_read(FUSB302_dev_t *dev, uint8_t address, uint8_t *data, uint8_t count)
{
    FUSB302_ret_t ret = dev->i2c_read(dev->i2c_address, address, data, count); // 调用板级注入的 I2C 读函数。
    if (ret != FUSB302_SUCCESS) {
        dev->err_msg = FUSB302_ERR_MSG("Fail to read register");
    }
    return ret;
}

static inline FUSB302_ret_t reg_write(FUSB302_dev_t *dev, uint8_t address, uint8_t *data, uint8_t count)
{
    FUSB302_ret_t ret = dev->i2c_write(dev->i2c_address, address, data, count); // 调用板级注入的 I2C 写函数。
    if (ret != FUSB302_SUCCESS) {
        dev->err_msg = FUSB302_ERR_MSG("Fail to write register");
    }
    return ret;
}

/** @brief 连续读取 6 次 BC_LVL，只有结果稳定一致才接受本次 CC 电平。 */
static FUSB302_ret_t FUSB302_read_cc_lvl(FUSB302_dev_t *dev, uint8_t * cc_value)
{
    /*  00: < 200 mV          : vRa
        01: >200 mV, <660 mV  : vRd-USB
        10: >660 mV, <1.23 V  : vRd-1.5
        11: >1.23 V           : vRd-3.0  */
    uint8_t cc, cc_verify;
    REG_READ(ADDRESS_STATUS0, &REG_STATUS0, 1);
    cc = REG_STATUS0 & BC_LVL_MASK;
    for (uint8_t i = 0; i < 5; i++) {
        REG_READ(ADDRESS_STATUS0, &REG_STATUS0, 1);
        cc_verify = REG_STATUS0 & BC_LVL_MASK;
        if (cc != cc_verify) {
            return FUSB302_BUSY;
        }
    }
    *cc_value = cc;
	return FUSB302_SUCCESS;
}

/**
 * @brief 有界重读CC电平，避免I2C异常时永久卡住PD任务。
 * @param dev FUSB302设备上下文。
 * @param cc_value 返回CC电平等级。
 * @return FUSB302_SUCCESS表示读取成功，其他值表示三次尝试后仍失败。
 * @note STEP9修复说明（相对官方基线）：官方代码使用无限while重读；当底层开始
 *       如实上传HAL错误后，该循环可能永远不退出。这里限制为三次，把错误交回上层。
 */
static FUSB302_ret_t FUSB302_read_cc_lvl_retry(FUSB302_dev_t *dev, uint8_t *cc_value)
{
    FUSB302_ret_t ret = FUSB302_BUSY;

    for (uint8_t i = 0; i < 3U; i++) {
        ret = FUSB302_read_cc_lvl(dev, cc_value);
        if (ret == FUSB302_SUCCESS) {
            return FUSB302_SUCCESS;
        }
        if (i < 2U) {
            dev->delay_ms(2);
        }
    }

    return ret;
}

/** @brief 从 RX FIFO 取 SOP token、消息头、Data Object 和 CRC，并产生收包事件。 */
static FUSB302_ret_t FUSB302_read_incoming_packet(FUSB302_dev_t *dev, FUSB302_event_t * events)
{
    uint8_t len, b[3];                                      // b[0] 是 SOP token，b[1..2] 是消息头。
    REG_READ(ADDRESS_FIFOS, b, 3);                          // 先读 token 和 16 位小端消息头。
    dev->rx_header = ((uint16_t)b[2] << 8) | b[1];          // 合成主机端 16 位消息头。
    len = (dev->rx_header >> 12) & 0x7;                     // 头部 bit14..12 给出 32 位对象数量。
    REG_READ(ADDRESS_FIFOS, dev->rx_buffer, len * 4 + 4);   // 每个对象 4 字节，最后再读出 4 字节 CRC。

    if (events) {
        *events |= FUSB302_EVENT_RX_SOP;
    }
    return FUSB302_SUCCESS;
}

/**
 * @brief 未连接状态下检测 VBUS，并依次测量 CC1/CC2 后选择 PD 收发引脚。
 */
static FUSB302_ret_t FUSB302_state_unattached(FUSB302_dev_t *dev, FUSB302_event_t * events)
{
    FUSB302_ret_t ret;

    REG_READ(ADDRESS_STATUS0, &REG_STATUS0, 1);
    if (REG_STATUS0 & VBUSOK) { // 官方寄存器 VBUSOK 置位后，才开始 CC 检测和接收机配置。
        /* enable internal oscillator */
        REG_POWER = PWR_BANDGAP | PWR_RECEIVER | PWR_MEASURE | PWR_INT_OSC;
        REG_WRITE(ADDRESS_POWER, &REG_POWER, 1);
        dev->delay_ms(2);

        /* read cc1 */
        REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC1; // CC1/CC2 保持 Rd，下次测量选择 CC1。
        REG_SWITCHES1 = SPECREV0;
        REG_MEASURE = 49;
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 3);
        dev->delay_ms(2);
        ret = FUSB302_read_cc_lvl_retry(dev, &dev->cc1);
        if (ret != FUSB302_SUCCESS) {
            return ret;
        }

        /* read cc2 */
        REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC2; // 改为测量 CC2。
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 1);
        dev->delay_ms(2);
        ret = FUSB302_read_cc_lvl_retry(dev, &dev->cc2);
        if (ret != FUSB302_SUCCESS) {
            return ret;
        }

        /* clear interrupt */
        REG_READ(ADDRESS_INTERRUPTA, &REG_INTERRUPTA, 2);
        dev->interrupta = 0;
        dev->interruptb = 0;

        /* enable tx on cc pin */
        if (dev->cc1 > 0) { // 插头方向落在 CC1：测量和 BMC 发送都选 CC1。
            REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC1;
            REG_SWITCHES1 = SPECREV0 | AUTO_CRC | TXCC1;
            //REG_SWITCHES1 = SPECREV0 | TXCC1;
        } else if (dev->cc2 > 0) { // 插头方向落在 CC2：测量和 BMC 发送都选 CC2。
            REG_SWITCHES0 = PDWN1 | PDWN2 | MEAS_CC2;
            REG_SWITCHES1 = SPECREV0 | AUTO_CRC | TXCC2;
            //REG_SWITCHES1 = SPECREV0 | TXCC2;
        } else {
            REG_SWITCHES0 = PDWN1 | PDWN2;
            REG_SWITCHES1 = SPECREV0;
        }
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 2);

        /* update state */
        dev->state = FUSB302_STATE_ATTACHED; // 后续 FUSB302_alert() 改走已连接处理函数。
        if (events) {
            *events |= FUSB302_EVENT_ATTACHED;
        }
    }
    return FUSB302_SUCCESS;
}

/**
 * @brief 已连接状态下汇总中断、检测掉线、处理 Hard Reset 和读取 RX FIFO。
 */
static FUSB302_ret_t FUSB302_state_attached(FUSB302_dev_t *dev, FUSB302_event_t * events)
{
    REG_READ(ADDRESS_STATUS0A, &REG_STATUS0A, 7); // 连续读状态和中断寄存器；中断寄存器读取后清除。
    dev->interrupta |= REG_INTERRUPTA;           // 先累计到软件位图，避免一次只消费一个事件。
    dev->interruptb |= REG_INTERRUPTB;
    if (dev->vbus_sense && ((REG_STATUS0 & VBUSOK) == 0)) { // 固定 PDO 模式下 VBUS 消失视为断开。
        /* reset cc pins to pull down */
        REG_SWITCHES0 = PDWN1 | PDWN2;
        REG_SWITCHES1 = SPECREV0;
        REG_MEASURE = 49;
        REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 3);

        /* turn off internal oscillator */
        REG_POWER = PWR_BANDGAP | PWR_RECEIVER | PWR_MEASURE;
        REG_WRITE(ADDRESS_POWER, &REG_POWER, 1);

        /* update state */
        dev->state = FUSB302_STATE_UNATTACHED; // 回到未连接状态，等待下一次 VBUS/CC 检测。
        if (events) {
            *events |= FUSB302_EVENT_DETACHED;
        }
        return FUSB302_SUCCESS;
    }
    if (REG_STATUS0A & HARDRST) {
        uint8_t reg_control = PD_RESET;
        REG_WRITE(ADDRESS_RESET, &reg_control, 1);
        return FUSB302_SUCCESS;
    }
    if (dev->interruptb & I_GCRCSENT) { // FUSB302 已自动发出 GoodCRC，协议层现在可以处理/回复收到的消息。
        dev->interruptb &= ~I_GCRCSENT;
        if (events) {
            *events |= FUSB302_EVENT_GOOD_CRC_SENT;
        }
    }
    if ((REG_STATUS1 & RX_EMPTY) == 0) { // RX FIFO 非空时读取一条消息。
        if (FUSB302_read_incoming_packet(dev, events) != FUSB302_SUCCESS) {
            uint8_t rx_flush = REG_CONTROL1 | RX_FLUSH;
            reg_write(dev, ADDRESS_CONTROL1, &rx_flush, 1);
        }
    }
    return FUSB302_SUCCESS;
}

/**
  * @brief  校验板级接口，复位并配置 FUSB302B 的 CC、自动重试、中断和电源模块。
  *
  * @param  dev : FUSB302 设备实例（用户须先填好 i2c_address/i2c_read/i2c_write/delay_ms）
  * @retval FUSB302_SUCCESS           成功
  * @retval FUSB302_ERR_PARAM         I2C 地址或读写回调为空
  * @retval FUSB302_ERR_READ_DEVICE   I2C 读器件 ID 失败（器件不存在/接线问题）
  * @retval FUSB302_ERR_DEVICE_ID     读到的器件版本位无效（不是 FUSB302）
  *
  * @note   初始化顺序：
  *           1) 校验用户移植接口（I2C 地址 + 读写回调）
  *           2) 读 DEVICE_ID 探测器件是否在线、版本位是否正确
  *           3) 初始化软件状态：UNATTACHED + 清空接收缓存
  *           4) SW_RES 软件全复位，让所有寄存器回到默认值
  *           5) 重新读回 15 个控制寄存器到本地缓存（REG_xxx 映射的基础）
  *           6) 配置 CC 下拉（Sink 角色）、PD 版本、测量阈值
  *           7) 配置发送自动重试 3 次
  *           8) 配置中断屏蔽（只放行关心的中断源）
  *           9) 打开 INT 总开关 + 给各模块上电
  */
FUSB302_ret_t FUSB302_init(FUSB302_dev_t *dev)
{
    /* ---- 步骤1：校验用户移植接口，缺一不可 ---- */
    if (dev->i2c_address == 0) {
        dev->err_msg = FUSB302_ERR_MSG("Invalid i2c address");
        return FUSB302_ERR_PARAM;
    }
    if (dev->i2c_read == 0) {
        dev->err_msg = FUSB302_ERR_MSG("Invalid i2c_read function");
        return FUSB302_ERR_PARAM;
    }
    if (dev->i2c_write == 0) {
        dev->err_msg = FUSB302_ERR_MSG("Invalid i2c_write function");
        return FUSB302_ERR_PARAM;
    }

    /* ---- 步骤2：读 DEVICE_ID 寄存器（0x01）探测器件 ----
     * 读失败=I2C 总线上找不到器件（接线/地址问题）。
     * 注意：这里先临时读入 reg_control[1]，仅用于下一步版本位校验；
     * 步骤5 会再从 reg_control[0] 起批量读回全部寄存器，届时此临时值被覆盖。 */
    if (reg_read(dev, ADDRESS_DEVICE_ID, &dev->reg_control[1], 1) != FUSB302_SUCCESS) {
        dev->err_msg = FUSB302_ERR_MSG("Device not found");
        return FUSB302_ERR_READ_DEVICE;
    }

    /* DEVICE_ID 的 bit7 是版本有效位（固定为 1），为 0 说明器件型号不对 */
	if ((dev->reg_control[1] & 0x80) == 0) {
        dev->err_msg = FUSB302_ERR_MSG("Invalid device version");
        return FUSB302_ERR_DEVICE_ID;
    }

    /* ---- 步骤3：初始化软件状态 ---- */
    dev->state = FUSB302_STATE_UNATTACHED; // 软件状态从未连接开始。
    dev->rx_header = 0;
    memset(dev->rx_buffer, 0, sizeof(dev->rx_buffer));

    /* ---- 步骤4：SW_RES 软件全复位（寄存器全部恢复默认值） ---- */
    /* restore default settings */
    REG_RESET = SW_RES; // 软件复位 FUSB302B 寄存器状态。
    REG_WRITE(ADDRESS_RESET, &REG_RESET, 1);

    /* ---- 步骤5：把 0x01~0x0F 共 15 个可读写寄存器一次性读回本地缓存，
     * 之后对 REG_xxx 的修改其实是先改这份缓存，再按需 REG_WRITE 同步到芯片 ---- */
    /* fetch all R/W registers */
    REG_READ(ADDRESS_DEVICE_ID, &REG_DEVICE_ID, 15);

    /* ---- 步骤6：配置 CC 开关、PD 版本、测量阈值 ---- */
    /* configure switchs and comparators */
    REG_SWITCHES0 = PDWN1 | PDWN2; // 作为 Sink，在 CC1/CC2 打开下拉 Rd。
    REG_SWITCHES1 = SPECREV0;      // SPECREV=01，即 PD 2.0 版本
    REG_MEASURE = 49;              // 测量比较器阈值（Slice 相关参数），用于 CC 电压判定
	REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 3); // 连续写 0x02~0x04 三个寄存器

    /* ---- 步骤7：配置发送失败自动重试 ---- */
    /* configure auto retries */
    REG_CONTROL3 &= ~N_RETRIES_MASK;                         // 先清掉重试次数字段
    REG_CONTROL3 |= N_RETRIES(3) | AUTO_RETRY; // 让芯片自动重试发送，次数字段设为 3。
    REG_WRITE(ADDRESS_CONTROL3, &REG_CONTROL3, 1);

    /* ---- 步骤8：配置中断屏蔽寄存器 Mask(0x0A) ----
     * 默认 0xFF 屏蔽全部，再逐个放开：VBUSOK/ACTIVITY/COLLISION/ALERT/CRC_CHK */
    /* configure interrupt mask */
    REG_MASK = 0xFF;
    REG_MASK &= ~(M_VBUSOK | M_ACTIVITY | M_COLLISION | M_ALERT | M_CRC_CHK);
    REG_WRITE(ADDRESS_MASK, &REG_MASK, 1);

    /* ---- 步骤8b：配置扩展中断屏蔽 Maska/Maskb ----
     * 放开：重发失败/硬复位已发/报文已发/收到硬复位/GoodCRC已发 */
    /* configure interrupt maska/maskb */
    REG_MASKA = 0xFF;
    REG_MASKA &= ~(M_RETRYFAIL | M_HARDSENT | M_TXSENT | M_HARDRST);
    REG_WRITE(ADDRESS_MASKA, &REG_MASKA, 1);
    REG_MASKB = 0xFF;
    REG_MASKB &= ~(M_GCRCSENT);
    REG_WRITE(ADDRESS_MASKB, &REG_MASKB, 1);

    /* ---- 步骤9：打开中断总开关 + 模块上电 ---- */
    /* enable interrupt */
    REG_CONTROL0 &= ~INT_MASK; // INT_MASK 清零，允许 INT_N 输出未屏蔽事件。
    REG_WRITE(ADDRESS_CONTROL0, &REG_CONTROL0, 1);

    /* Power on, enable VUSB detection
     * 给带隙基准、接收器、测量块上电（内部振荡器 PWR_INT_OSC 未开，由硬件按需起振） */
    REG_POWER = PWR_BANDGAP | PWR_RECEIVER | PWR_MEASURE;
    REG_WRITE(ADDRESS_POWER, &REG_POWER, 1);

    dev->vbus_sense = 1;                        // 软件记录 VBUS 检测当前处于使能状态
    dev->err_msg = FUSB302_ERR_MSG("");
	return FUSB302_SUCCESS;
}

/**
 * @brief 清除一次 PD 连接的软件状态，下一轮重新检测 VBUS 和 CC。
 * @param dev FUSB302 设备上下文。
 * @return FUSB302_SUCCESS；dev 为空时返回 FUSB302_ERR_PARAM。
 * @note 不读写 I2C，不清 reg_control，避免破坏初始化好的寄存器配置缓存。
 * @note STEP9修复说明：该函数专门清掉上一轮ATTACHED、CC方向、接收缓存和中断缓存，
 *       使下一次START从UNATTACHED重新检测；这是重复进入PD/PPS页面能够恢复的关键边界。
 */
FUSB302_ret_t FUSB302_session_reset(FUSB302_dev_t *dev)
{
    if (dev == 0) {
        return FUSB302_ERR_PARAM;
    }

    dev->err_msg = 0;
    dev->rx_header = 0;
    memset(dev->rx_buffer, 0, sizeof(dev->rx_buffer));
    memset(dev->reg_status, 0, sizeof(dev->reg_status));
    dev->interrupta = 0;
    dev->interruptb = 0;
    dev->cc1 = 0;
    dev->cc2 = 0;
    dev->state = FUSB302_STATE_UNATTACHED;
    dev->vbus_sense = 1;

    return FUSB302_SUCCESS;
}

/**
  * @brief  触发一次 PD 逻辑复位（只复位 PD 协议状态机，不影响寄存器配置）。
  * @note   与 init 中的 SW_RES 不同：SW_RES 是全芯片软件复位，PD_RESET 仅清协议状态。
  *         发完 Hard Reset 后也会调用，确保双方协议层重新从初始状态开始。
  */
FUSB302_ret_t FUSB302_pd_reset(FUSB302_dev_t *dev)
{
    uint8_t reg = PD_RESET;
    REG_WRITE(ADDRESS_RESET, &reg, 1);
    return FUSB302_SUCCESS;
}

/**
  * @brief  使能/关闭 CC1/CC2 上的下拉电阻 Rd（Sink 识别电阻）。
  * @param  enable : 1=开启下拉（宣告自己是 Sink/用电设备）；0=关闭下拉
  * @note   Source 端内部是上拉 Rp，Sink 端必须呈现 Rd，Source 检测到 Rd 才会输出电压。
  */
FUSB302_ret_t FUSB302_pdwn_cc(FUSB302_dev_t *dev, uint8_t enable)
{
    REG_SWITCHES0 = enable ? (PDWN1 | PDWN2) : 0;
	REG_WRITE(ADDRESS_SWITCHES0, &REG_SWITCHES0, 1);
    return FUSB302_SUCCESS;
}

/** @brief 打开或关闭 VBUSOK 中断参与掉线判断；PPS 低电压时上层会关闭它。
  *
  * @param  enable : 1=使能 VBUSOK 中断（普通 Fixed 电压档用）
  *                  0=屏蔽 VBUSOK 中断（请求 PPS 电压 <4V 时必须关闭）
  * @note   原因：FUSB302 的 VBUS 检测低阈值约 4V，PPS 可以请求到 3.3V，
  *         此时 VBUSOK 会一直为 0，若不屏蔽会被误判成"VBUS 掉电/掉线"。
  *         只有状态真正变化时才写寄存器，避免多余 I2C 操作。
  */
FUSB302_ret_t FUSB302_set_vbus_sense(FUSB302_dev_t *dev, uint8_t enable)
{
    if (dev->vbus_sense != enable) {
        if (enable) {
            REG_MASK &= ~M_VBUSOK;  /* enable VBUSOK interrupt：屏蔽位写 0 = 放开中断 */
        } else {
            REG_MASK |= M_VBUSOK;   /* disable VBUSOK interrupt：屏蔽位写 1 = 屏蔽中断 */
        }
        REG_WRITE(ADDRESS_MASK, &REG_MASK, 1);
        dev->vbus_sense = enable;   // 同步软件标志
    }
    return FUSB302_SUCCESS;
}

/**
  * @brief  读取 FUSB302 芯片的版本号与修订号。
  * @param  version_ID  : 输出，芯片版本号（DEVICE_ID 的 bit6~4，共 3 位）
  * @param  revision_ID : 输出，芯片修订号（DEVICE_ID 的 bit3~0，共 4 位）
  * @retval FUSB302_SUCCESS 成功；FUSB302_ERR_PARAM 器件无效或 dev 为空
  * @note   允许传入 NULL 表示只取其中一个字段。
  */
FUSB302_ret_t FUSB302_get_ID(FUSB302_dev_t *dev, uint8_t * version_ID, uint8_t * revision_ID)
{
    if (dev && (REG_DEVICE_ID & 0x80)) {   // 器件有效位必须为 1
        if (version_ID) {
            *version_ID = (REG_DEVICE_ID >> 4) & 0x7;   // bit6~4 → 版本号
        }
        if (revision_ID) {
            *revision_ID = (REG_DEVICE_ID >> 0) & 0xF;  // bit3~0 → 修订号
        }
        return FUSB302_SUCCESS;
    }
    return FUSB302_ERR_PARAM;
}

/**
  * @brief  获取最近一次状态机检测到的 CC1/CC2 电压等级。
  * @param  cc1 : 输出，CC1 等级（0~3，对应 BC_LVL 区间，数值越大说明对端 Rp 电流能力越强）
  * @param  cc2 : 输出，CC2 等级
  * @note   读的是软件缓存（由 unattached/attached 状态处理函数轮询更新），不产生 I2C 通信。
  *         哪个 CC 有非零值就说明线从哪个方向插入。
  */
FUSB302_ret_t FUSB302_get_cc(FUSB302_dev_t *dev, uint8_t *cc1, uint8_t *cc2)
{
    if (cc1) {
        *cc1 = dev->cc1;
    }
    if (cc2) {
        *cc2 = dev->cc2;
    }
	return FUSB302_SUCCESS;
}

/**
  * @brief  实时读取 VBUS 电压是否有效（>4V）。
  * @param  vbus : 输出，1=VBUSOK（VBUS 高于阈值）；0=VBUS 无效/低于 4V
  * @note   直接读 STATUS0 寄存器（非缓存），用于确认对端是否已把电压调到请求值。
  */
FUSB302_ret_t FUSB302_get_vbus_level(FUSB302_dev_t *dev, uint8_t *vbus)
{
    uint8_t reg_control;
    REG_READ(ADDRESS_STATUS0, &reg_control, 1);
    *vbus = reg_control & VBUSOK ? 1 : 0;
	return FUSB302_SUCCESS;
}

/**
  * @brief  取出最近一次收到的 SOP 报文（报文头 + 数据对象）。
  * @param  header : 输出，16 位 PD 消息头
  * @param  data   : 输出，数据对象数组（调用者需保证缓冲区至少 7*4 字节）
  * @note   数据来源是软件缓存：attached 状态处理函数检测到 RX_SOP 事件后，
  *         已经把 RX FIFO 的内容解析进 dev->rx_header / rx_buffer。
  *         报文头 bit14~12 是数据对象个数（0~7），据此决定拷贝多少字节。
  */
FUSB302_ret_t FUSB302_get_message(FUSB302_dev_t *dev, uint16_t * header, uint32_t * data)
{
    if (header) {
        *header = dev->rx_header;
    }
    if (data) {
        uint8_t len = (dev->rx_header >> 12) & 0x7;  // 消息头 bit14~12 = Data Object 数量
        memcpy(data, dev->rx_buffer, len * 4);        // 每个对象 4 字节，小端拷贝
    }
	return FUSB302_SUCCESS;
}

/**
 * @brief 把一条 SOP 消息编码为 FUSB302 TX FIFO token 序列并启动发送。
 * @param dev 设备上下文。
 * @param header 16 位 PD 消息头。
 * @param data 0~7 个 32 位 Data Object；对象数由 header 给出。
 * @return FUSB302_SUCCESS 或寄存器写错误。
 */
FUSB302_ret_t FUSB302_tx_sop(FUSB302_dev_t *dev, uint16_t header, const uint32_t *data)
{
    uint8_t buf[40];                              // 最大 7 个对象时仍能容纳全部 token 和数据。
    uint8_t * pbuf = buf;                         // 始终指向下一个待写位置。
    uint8_t obj_count = ((header >> 12) & 7);     // 从消息头取 Data Object 数量。
    *pbuf++ = (uint8_t)TX_TOKEN_SOP1;             // 依次写入 SOP 起始 token。
    *pbuf++ = (uint8_t)TX_TOKEN_SOP1;
    *pbuf++ = (uint8_t)TX_TOKEN_SOP1;
    *pbuf++ = (uint8_t)TX_TOKEN_SOP2;
    *pbuf++ = (uint8_t)TX_TOKEN_PACKSYM | ((obj_count << 2) + 2); // PACKSYM 长度=头2字节+对象字节数。
    *pbuf++ = header & 0xFF; header >>= 8;         // PD 消息头低字节先入 FIFO。
    *pbuf++ = header & 0xFF;                       // 再写消息头高字节。
    for (uint8_t i = 0; i < obj_count; i++) {     // 逐个写 32 位 Data Object。
        uint32_t d = *data++;                     // 取当前对象，同时移动到下一个对象。
        *pbuf++ = d & 0xFF; d >>= 8;              // 小端顺序写第 0 字节。
        *pbuf++ = d & 0xFF; d >>= 8;              // 第 1 字节。
        *pbuf++ = d & 0xFF; d >>= 8;              // 第 2 字节。
        *pbuf++ = d & 0xFF;                       // 第 3 字节。
    }
    *pbuf++ = (uint8_t)TX_TOKEN_JAM_CRC;           // 让芯片追加 CRC。
    *pbuf++ = (uint8_t)TX_TOKEN_EOP;               // 写入包结束 token。
    *pbuf++ = (uint8_t)TX_TOKEN_TXOFF;             // 按 FUSB302 token 流结束发送。
    *pbuf++ = (uint8_t)TX_TOKEN_TXON;              // 启动本次 token 序列发送。
    REG_WRITE(ADDRESS_FIFOS, buf, pbuf - buf);      // 一次性把完整 token 序列写入 TX FIFO。
    dev->delay_ms(2);                              // 给硬件发送留出短暂时间。
	return FUSB302_SUCCESS;
}

/**
  * @brief  发送 USB-PD Hard Reset（硬复位）并复位本地 PD 协议状态机。
  *
  * @param  dev : FUSB302 设备实例
  * @retval FUSB302_SUCCESS
  *
  * @note   硬复位的作用与流程：
  *           - Hard Reset 是 PD 协议中最高级别的错误恢复手段，用于协商失败、
  *             协议超时、对端无响应等异常场景，强制双方回到默认 5V/USB-default 状态。
  *           - 执行步骤：
  *               1) 读回 CONTROL3 当前值（用 REG_CONTROL3 缓存），置 SEND_HARDRESET 位，
  *                  芯片随即在 CC 线上自动发出 Hard Reset 序列（tHardReset 波形）
  *               2) 延时 6ms，等待整个硬复位序列在物理层发送完毕
  *                  （协议规定 Hard Reset 含两段 reset 信号 + 等待，需数毫秒）
  *               3) 写 PD_RESET 复位本机 PD 逻辑状态机，使本地协议层也回到初始态，
  *                  之后可重新等待对端 Source_Capabilities 发起协商
  *           - 注意这里不清空 CONTROL3：SEND_HARDRESET 是触发位，硬件发完会自动清。
  */
FUSB302_ret_t FUSB302_tx_hard_reset(FUSB302_dev_t *dev)
{
    uint8_t reg_control = REG_CONTROL3;      // 取 CONTROL3 缓存值（保留其他配置如重试次数）
    reg_control |= SEND_HARDRESET;           // 置位 bit6，触发芯片发送 Hard Reset 序列
    REG_WRITE(ADDRESS_CONTROL3, &reg_control, 1);
    dev->delay_ms(6);                        // 等待硬复位波形完整发出（约需 6ms）
    reg_control = PD_RESET;                  // 准备 PD 逻辑复位命令（不复位寄存器配置）
    REG_WRITE(ADDRESS_RESET, &reg_control, 1);
    return FUSB302_SUCCESS;
}

/**
  * @brief  按当前连接状态分派事件处理函数（状态机分派器），是上层轮询 FUSB302 的统一入口。
  *
  * @param  dev    : FUSB302 设备实例指针
  * @param  events : 输出参数，返回本次轮询检测到的事件位图（ATTACHED/DETACHED/RX_SOP/...）
  * @retval        : FUSB302_SUCCESS 或错误码
  *
  * @note   工作原理 —— 函数指针查找表（替代 switch-case 的经典状态机写法）：
  *           1) 定义一个常量函数指针数组 handler[]，按状态枚举值顺序存放两个处理函数：
  *                handler[0] = FUSB302_state_unattached （未连接态：查 CC 电平，判断是否插入）
  *                handler[1] = FUSB302_state_attached    （已连接态：读中断，处理收报文/掉线）
  *           2) 用 dev->state 直接作数组下标，调用当前状态对应的处理函数。
  *              状态与处理函数一一对应，新增状态时只需在数组里追加一项。
  *           3) 若 state 值越界（异常情况），强制回到 UNATTACHED 安全初始态。
  *
  *         上层（PDUFPTask）周期性调用本函数即可，无需关心芯片当前处于哪种连接状态。
  */
FUSB302_ret_t FUSB302_alert(FUSB302_dev_t *dev, FUSB302_event_t * events)
{
    /* 函数指针表：下标 = 状态值（对应 enum FUSB302_state_t），内容 = 该状态的处理函数。
     * const 保证表本身不可被运行时修改，可存放在 Flash 中。 */
    FUSB302_ret_t (* const handler[]) (FUSB302_dev_t *, FUSB302_event_t *) = {
        FUSB302_state_unattached,   // [0] 未连接状态处理函数
        FUSB302_state_attached      // [1] 已连接状态处理函数
    };
    /* 边界检查：state 必须是合法下标（0 或 1），避免数组越界调用 */
    if (dev->state < sizeof(handler) / sizeof(handler[0])) {
        /* 查表并调用：handler[dev->state] 取出函数指针，再以 (dev, events) 为参数执行 */
        return handler[dev->state](dev, events);
    }
    /* 异常恢复：state 为非法值（如内存被踩坏）时，复位到未连接态，下次轮询从安全状态重来 */
    dev->state = FUSB302_STATE_UNATTACHED;
    return FUSB302_SUCCESS;
}
