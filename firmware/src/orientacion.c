#include "sapi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "imu.h"
#include "lcd.h"
#include "wifi.h"

#define IMU_STACK_SIZE 512
#define LCD_STACK_SIZE 512
#define WIFI_STACK_SIZE 512

StackType_t imuTaskStack[IMU_STACK_SIZE], lcdTaskStack[LCD_STACK_SIZE], wifiTaskStack[WIFI_STACK_SIZE];
StaticTask_t imuTaskTCB, lcdTaskTCB, wifiTaskTCB;

int main( void ) {
    /* ------------- INICIALIZACIONES ------------- */

    boardConfig();


    taskLCD_Init();

    /* ------------- TASKS ------------- */

    xTaskCreateStatic( taskIMU, "TaskIMU", IMU_STACK_SIZE, NULL,
                        tskIDLE_PRIORITY+10, imuTaskStack, &imuTaskTCB);
    xTaskCreateStatic( taskLCD, "TaskLCD", LCD_STACK_SIZE, NULL,
                        tskIDLE_PRIORITY+1, lcdTaskStack, &lcdTaskTCB);
    xTaskCreateStatic( taskWiFi, "TaskWiFi", WIFI_STACK_SIZE, NULL,
                        tskIDLE_PRIORITY+1, wifiTaskStack, &wifiTaskTCB);

    vTaskStartScheduler();

    while(1);

    /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
        por ningun S.O. */
    return 0 ;
}
