#ifndef __USER_CMDSTRATEGY_H__
#define __USER_CMDSTRATEGY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define CMD_PROTOCOL_VERSION     1U     // 当前命令协议版本；请求帧版本不等于它时，板子拒绝执行。

/* 上位机请求命令号；板子回复时会把命令号按位或 0x80，例如 0x11 回复为 0x91。 */
#define CMD_CAL_GET              0x10U  // 查询当前三档 ADC 校准参数。
#define CMD_RANGE_GET            0x11U  // 查询量程模式和当前实际硬件档位。
#define CMD_RANGE_SET            0x12U  // 设置量程模式：AUTO、LOW、MID 或 HIGH。
#define CMD_CAL_SET              0x13U  // 下发并保存三档 ADC 校准参数。
#define CMD_CAL_RESET            0x14U  // 将 ADC 校准参数恢复默认值；载荷需携带确认码。
#define CMD_FW_UPDATE            0x15U  // 请求进入固件升级流程；载荷需携带确认码。

/**
 * @brief 初始化命令接收缓存
 * @note 上电后由 MessageReceiveTask 调用一次，清空未处理的旧字节。
 */
void CmdStrategy_Init(void);

/**
 * @brief 接收一段 USB 字节并尝试解析、执行完整命令帧
 * @param data USB CDC 本次收到的原始字节地址
 * @param length 本次收到的字节数
 * @note 一次调用不一定是一整帧命令；函数内部会缓存、拼包并处理多帧。
 */
void CmdStrategy_ProcessRx(const uint8_t *data, uint16_t length);

/**
 * @brief 在任务上下文执行已确认的命令
 */
void CmdStrategy_Process(void);

#ifdef __cplusplus
}
#endif

#endif
