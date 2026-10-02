/**
 * @file user_TasksInit.c
 * @brief 集中创建 PowerPico 的用户任务和消息队列。
 *
 * 谁写谁读：
 * - KeyTask -> Key_MessageQueue -> LvHandlerTask
 * - UI -> PD_cmd_MessageQueue -> PDUFPTask
 * - PDUFPTask -> PD_handle_event_MsgQueue -> LvHandlerTask
 * - MessageSendTask -> PowerDataQueue -> LvHandlerTask
 * - USB 接收回调 -> CmdRxQueue -> MessageReceiveTask
 * - MessageReceiveTask -> CmdTxQueue -> MessageSendTask -> USB CDC
 *
 * 本文件只负责“建任务、建队列”，各任务的业务循环在对应的 task.c 中。
 
   注意以上是源码的注释，现在是用来临时测试用的！！，后续工程完成后，继续沿用以上相关注释
 */

 /**
 * @file user_TasksInit.c
 * @brief 集中创建当前阶段任务和消息队列(临时)
 *
 * 当前职责：
 * 1. 创建六条官方架构使用的消息队列；
 * 2. 创建现阶段需要的六个任务；
 * 3. 记录创建失败的具体对象；
 * 4. 不在这里执行ADC、LCD、USB或PD业务。
 */


/* Private includes -----------------------------------------------------------*/
//includes
#include "user_TasksInit.h"
#include "key.h"

//tasks
#include "user_MessageTask.h"
#include "user_LVGLTask.h"


/* -------------------- 任务句柄 -------------------- */

osThreadId_t HardwareInitTaskHandle;
osThreadId_t MessageSendTaskHandle;
osThreadId_t MessageReceiveTaskHandle;
osThreadId_t KeyTaskHandle;
osThreadId_t PDUFPTaskHandle;
osThreadId_t LvHandlerTaskHandle;


/* -------------------- 消息队列句柄 -------------------- */
osMessageQueueId_t Key_MessageQueue;
osMessageQueueId_t PD_cmd_MessageQueue;
osMessageQueueId_t PD_handle_event_MsgQueue;
osMessageQueueId_t PowerDataQueue;
osMessageQueueId_t CmdRxQueue;
osMessageQueueId_t CmdTxQueue;


/*
 * 创建失败位图。
 * 每一位对应一个任务或队列，方便只看LCD时定位问题。
 */
static uint32_t s_create_error_mask;

/*
 * 防止User_Tasks_Init()被重复调用。
 * 重复创建任务和队列会浪费FreeRTOS堆内存。
 */
static bool s_tasks_init_called;

/*
 * HardwareInitTask运行后置true。
 * 该变量参与LCD状态判断，不是Keil Watch专用变量。
 */
static volatile bool s_hardware_init_done;

/* -------------------- 任务属性 -------------------- */

/*
 * STEP6B的HardwareInitTask暂时只验证：
 * 高优先级运行一次，然后主动删除自己。
 * 因此512字节栈已经足够。
 */
static const osThreadAttr_t HardwareInitTask_attributes = 
{
  .name = "HwInitTask",
  .stack_size = 128*10 ,
  .priority = osPriorityHigh
};


/*
 * MessageSendTask就是原来的adcProcessTask。
 * 现阶段处理逻辑和1024字节栈大小都保持不变。
 */
static const osThreadAttr_t MessageSendTask_attributes = 
{
  .name = "MessageSendTask",
  .stack_size = 1024,
  .priority = osPriorityAboveNormal
};


/* STEP6E负责USB命令拼包和解析，给足1KB任务栈。 */
static const osThreadAttr_t MessageReceiveTask_attributes = 
{
  .name = "MsgRecvTask",
  .stack_size = 1024U,
  .priority = osPriorityNormal
};

/* 后续用于按键扫描，本轮不读取GPIO。 */
static const osThreadAttr_t KeyTask_attributes =
{
    .name = "KeyTask",
    .stack_size = 512U,
    .priority = osPriorityBelowNormal
};

/*
 * STEP9修复说明（相对当前复刻工程原配置）：
 * PD任务比LVGL高一个CMSIS优先级，恢复官方工程“PD略高于界面”的相对关系，
 * 减少LCD刷新抢占10 ms阻塞式I2C事务的机会。实板结果支持该调整有效，
 * 但本轮未做单变量A/B，因此不把任务优先级单独认定为唯一根因。
 */
static const osThreadAttr_t PDUFPTask_attributes =
{
    .name = "PDUFPTask",
    .stack_size = 128*16,
    .priority = osPriorityBelowNormal1
};

/* STEP6D中唯一允许访问LCD的任务。 */
static const osThreadAttr_t LvHandlerTask_attributes =
{
    .name = "LvHandlerTask",
    .stack_size = 128U * 48U,
    .priority = osPriorityBelowNormal
};


/**
 * @brief 创建当前阶段任务和消息队列
 */
void User_Tasks_Init(void)
{

      if (s_tasks_init_called)
      {
        /*
         * 本函数只能调用一次。
         * 如果误调用第二次，直接返回，不重复申请FreeRTOS堆内存。
         */
          return;
      }
        s_tasks_init_called = true;
        s_create_error_mask = 0U;
        s_hardware_init_done = false;


    /*
     * 先创建队列，再创建任务。
     *
     * 原因：
     * 调度器启动后，高优先级任务可能马上运行。
     * 必须保证任务运行前，它依赖的队列已经存在。
     */
    Key_MessageQueue = osMessageQueueNew(4,sizeof(key_event_t),NULL);

    PD_cmd_MessageQueue =
        osMessageQueueNew(4U, sizeof(PD_command_msg_t), NULL);

    PD_handle_event_MsgQueue =
        osMessageQueueNew(4U, sizeof(PD_handle_event_t), NULL);

    PowerDataQueue =
        osMessageQueueNew(8U, sizeof(PowerData_T), NULL);

    CmdRxQueue =
        osMessageQueueNew(CMD_RX_QUEUE_DEPTH,
                          sizeof(CmdRxChunk_T),
                          NULL);

    CmdTxQueue =
        osMessageQueueNew(CMD_TX_QUEUE_DEPTH,
                          sizeof(CmdTxFrame_T),
                          NULL);

    

    /* 逐个检查队列，生成LCD可观察的错误位图。 */
    if (Key_MessageQueue == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_KEY_QUEUE;
    }
    

    if (PD_cmd_MessageQueue == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_PD_CMD_QUEUE;
    }

    if (PD_handle_event_MsgQueue == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_PD_EVT_QUEUE;
    }

    if (PowerDataQueue == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_POWER_QUEUE;
    }

    if (CmdRxQueue == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_CMD_RX_QUEUE;
    }

    if (CmdTxQueue == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_CMD_TX_QUEUE;
    }


  /*
   * 创建任务。
   * STEP6D开始由LvHandlerTask独占LCD。
   * defaultTask不再直接调用LCD绘图函数。
   * 本轮也没有保留adcProcessTask：
   * 原ADC任务直接改名为MessageSendTask。
   */

    HardwareInitTaskHandle =
        osThreadNew(HardwareInitTask, NULL, &HardwareInitTask_attributes);

    MessageSendTaskHandle =
        osThreadNew(MessageSendTask, NULL, &MessageSendTask_attributes);

    MessageReceiveTaskHandle =
        osThreadNew(MessageReceiveTask, NULL, &MessageReceiveTask_attributes);

    KeyTaskHandle =
        osThreadNew(KeyTask, NULL, &KeyTask_attributes);

    PDUFPTaskHandle =
        osThreadNew(PDUFPTask, NULL, &PDUFPTask_attributes);

    LvHandlerTaskHandle =
        osThreadNew(LvHandlerTask, NULL, &LvHandlerTask_attributes);

    /*
     * 任务创建失败时记录对应错误位。
     * LCD会把该位图显示成BERR xxxx，便于直接定位失败对象。
     */
    if (HardwareInitTaskHandle == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_HW_TASK;
    }

    if (MessageSendTaskHandle == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_MSG_SEND_TASK;
    }

    if (MessageReceiveTaskHandle == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_MSG_RECV_TASK;
    }

    if (KeyTaskHandle == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_KEY_TASK;
    }

    if (PDUFPTaskHandle == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_PD_TASK;
    }

    if (LvHandlerTaskHandle == NULL)
    {
        s_create_error_mask |= USER_TASK_ERR_LVGL_TASK;
    }
}



 /**
 * @brief 标记HardwareInitTask已经运行
 */
void User_Tasks_MarkHardwareInitDone(void)
{
    s_hardware_init_done = true;
}



/**
 * @brief 获取任务框架当前状态
 */
UserTasksStatus_T User_Tasks_GetStatus(void)
{
    if (s_create_error_mask != 0U)
    {
        return USER_TASKS_STATUS_CREATE_FAIL;
    }

    if (!s_hardware_init_done)
    {
        return USER_TASKS_STATUS_WAIT;
    }

    return USER_TASKS_STATUS_READY;
}
