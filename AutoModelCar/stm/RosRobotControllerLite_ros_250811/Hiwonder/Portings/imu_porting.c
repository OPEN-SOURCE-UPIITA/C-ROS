#include "global.h"
#include "lwmem_porting.h"
#include "packet_reports.h"
#include "global_conf.h"
#include "QMI8658.h"

#if ENABLE_IMU
/**
 * @brief  imu task 入口函数
 *
 */
 
/*
void imu_task_entry(void *argument)
{
    extern osSemaphoreId_t mpu6050_data_readyHandle;
	extern osMutexId_t oled_mutexHandle;

    imus_init();
    imus[0]->reset(imus[0]);

    PacketReportIMU_Raw_TypeDef report;
    for(;;) {
//        osSemaphoreAcquire(mpu6050_data_readyHandle, osWaitForever);
//		osMutexAcquire(oled_mutexHandle, osWaitForever);
        imus[0]->update(imus[0]);
//		osMutexRelease(oled_mutexHandle);
		imus[0]->get_accel(imus[0], report.array.accel_array);
		imus[0]->get_gyro(imus[0], report.array.gyro_array);
        printf("%d,%d,%d\r\n",(int)report.array.accel_array[0],(int)report.array.accel_array[1],(int)report.array.accel_array[2]);
//        packet_transmit(&packet_controller, PACKET_FUNC_IMU, &report, sizeof(PacketReportIMU_Raw_TypeDef));
        osDelay(500);
    }
}
*/

struct QMI8658 qmi8658;

void imu_task_entry(void *argument)
{
    extern osSemaphoreId_t mpu6050_data_readyHandle;
	extern osMutexId_t oled_mutexHandle;

    if(begin() == 0)
    {
//        printf("qmi8658_init fail");
    }

    PacketReportIMU_Raw_TypeDef report;
    for(;;) {
        osSemaphoreAcquire(mpu6050_data_readyHandle, osWaitForever);
        read_xyz(report.array.accel_array,report.array.gyro_array);
//        printf("%x%x%x",0x11,0x11,0x11);
//        printf("%d,%d,%d\r\n",(int)report.array.accel_array[0],(int)report.array.accel_array[1],(int)report.array.accel_array[2]);
        packet_transmit(&packet_controller, PACKET_FUNC_IMU, &report, sizeof(PacketReportIMU_Raw_TypeDef));
        osDelay(20);
    }
}
#endif

