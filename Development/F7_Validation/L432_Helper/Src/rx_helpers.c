/**
  *****************************************************************************
  * @file    rx_helpers.c
  * @author  Tyler
  *
  * @defgroup RX_Helpers RX Helpers
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include <string.h>

#include "usart.h"

#include "rx_helpers.h"
#include "helpers.h"
#include "types.h"




/********************************** Macros ***********************************/
#define NOTIFIED_FROM_RX_ISR 0x40
#define NUM_MOTORS 18

#define INST_WRITE_DATA      0x03
#define INST_READ_DATA       0x02
#define REG_GOAL_POSITION    0x1E
#define REG_CURRENT_POSITION 0x24




/****************************** Public Variables *****************************/
extern osThreadId RXHandle;
extern osMessageQId toBeSentQHandle;




/***************************** Private Variables *****************************/
static uint16_t motorDataTable[NUM_MOTORS] = {0};
static volatile uint8_t buff[9] = {0};




/************************ Private Function Prototypes ************************/
static void writeToMotorDataTable(uint8_t id, uint16_t data){
    if(id <= NUM_MOTORS){
        motorDataTable[id] = data;
    }
}

static void processWriteDataInst(){
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&buff[8], 1); // CHKSM
    bool statusIsOkay = waitUntilNotifiedOrTimeout(NOTIFIED_FROM_RX_ISR, 1);

    uint8_t computedChecksum = Dynamixel_ComputeChecksum(
            (uint8_t*)buff,
            sizeof(buff)
    );

    bool checksumIsValid = (computedChecksum == buff[8]);

    if(checksumIsValid){
        uint8_t address = buff[5];
        uint8_t id = buff[2];

        switch(address){
            case REG_GOAL_POSITION:
                writeToMotorDataTable(id, buff[6] | (buff[7] << 8));
                break;
            default:
                break;
        }
    }
}

static void processReadDataInst(){
    uint8_t computedChecksum = Dynamixel_ComputeChecksum((uint8_t*)buff, 8);
    bool checksumIsValid = (computedChecksum == buff[7]);

    uint8_t id = buff[2];
    if(checksumIsValid && (id <= NUM_MOTORS)){
        bool addressIsValid = false;
        uint8_t address = buff[5];
        Data_t data;
        data.id = id;

        switch(address){
            case REG_CURRENT_POSITION:
                data.pos = motorDataTable[id];
                addressIsValid = true;
                break;
            default:
                break;
        }
        if(addressIsValid){
            osDelay(pdMS_TO_TICKS(1));
            xQueueSend(toBeSentQHandle, &data, 0);
        }
    }
}




/******************************** Functions **********************************/
bool receive(){
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)buff, 8);
    bool status = waitUntilNotifiedOrTimeout(NOTIFIED_FROM_RX_ISR, 2);

    if(!status){
       HAL_UART_AbortReceive(&huart1);
       memset((uint8_t*)buff, 0, sizeof(buff));
    }

    return status;
}

void processData(){
    uint8_t instruction = buff[4];

    switch(instruction){
        case INST_WRITE_DATA:
            processWriteDataInst();
            break;
        case INST_READ_DATA:
            processReadDataInst();
            break;
        default:
            break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(huart == &huart1){
        xTaskNotifyFromISR(
            RXHandle,
            NOTIFIED_FROM_RX_ISR,
            eSetBits,
            &xHigherPriorityTaskWoken
        );
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
