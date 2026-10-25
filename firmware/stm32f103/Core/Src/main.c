/** M1 固件入口。前台组装 board/runtime/telemetry，不包含控制算法。
 * 初始化先禁止调制并建立 BOOT，再配置外设，进入 DISABLED 后才启动采样事件。
 * 永无通电对齐、默认电流或 PWM 使能；停止调制不等于驱动桥高阻断能。 */
#include "main.h"
#include "board_io.h"
#include "motor_runtime.h"
#include "telemetry.h"

/** 复位后的唯一入口，无参数，正常不返回；失败锁存 FAULT 并保持 PWM 禁止。 */
int main(void)
{
    HAL_Init();
    motor_runtime_init();
    bool devices_are_ready = board_initialize_devices();
    motor_runtime_finish_boot(devices_are_ready);
    if (!devices_are_ready) {
        Error_Handler();
    }
    if (!board_start_sampling()) {
        motor_runtime_latch_fault(MOTOR_FAULT_INITIALIZATION);
    }
    while (1) {
        motor_runtime_check_deadline(board_get_time_us());
        telemetry_poll_status(HAL_GetTick());
    }
}

void Error_Handler(void)
{
    motor_runtime_latch_fault(MOTOR_FAULT_INITIALIZATION);
    while (1) {
        board_disable_output();
    }
}
