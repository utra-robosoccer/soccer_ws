/**
  *****************************************************************************
  * @file    UartDriver.h
  * @author  Tyler Gamvrelis
  * @brief   TODO -- brief description of file
  *
  * @defgroup Header
  * @ingroup  UART
  * @{
  *****************************************************************************
  */




#ifndef __UART_DRIVER_H__
#define __UART_DRIVER_H__




/********************************* Includes **********************************/
#include "HalUartInterface.h"

#ifdef THREADED
#include "cmsis_os.h"
#include "Notification.h"
#endif




/******************************* uart_interface ******************************/
namespace uart{
// Classes and structs
// ----------------------------------------------------------------------------
template <class UARTInterface>
class UartDriver{
public:
    UartDriver();
    ~UartDriver() {}

    void setUartPtr(UART_HandleTypeDef* uartHandlePtr);
    void setIOType(IO_Type io_type);
    IO_Type getIOType(void) const;

    // TODO: if we have an err_t type, we could use it to indicate both HAL and
    // FreeRTOS issues, so here that would be good as the ideal return type would
    // be able to indicate both an issue with the hardware AND with the OS (e.g. a
    // timeout)
    bool transmit(uint8_t* arrTransmit, size_t numBytes);
    bool receive(uint8_t* arrReceive, size_t numBytes);

private:
    IO_Type io_type = IO_Type::POLL;
    UARTInterface* hw_if = nullptr;
#ifdef THREADED
    static constexpr uint32_t TRANSFER_TIMEOUT = pdMS_TO_TICKS(2);
    static constexpr TickType_t MAX_BLOCK_TIME = pdMS_TO_TICKS(2);
#endif
#ifndef THREADED
    constexpr uint32_t TRANSFER_TIMEOUT = 2;
#endif
};

// Public
// ----------------------------------------------------------------------------
template <class UARTInterface> UartDriver<UARTInterface>::UartDriver(){

}

template <class UARTInterface> void UartDriver<UARTInterface>::setUartPtr(
    UART_HandleTypeDef* uartHandlePtr
)
{
    hw_if->setUartPtr(uartHandlePtr);
}

template <class UARTInterface> void UartDriver<UARTInterface>::setIOType(IO_Type io_type){
    this->io_type = io_type;
}

template <class UARTInterface> IO_Type UartDriver<UARTInterface>::getIOType(void) const{
    return this->io_type;
}

template <class UARTInterface> bool UartDriver<UARTInterface>::transmit(
    uint8_t* arrTransmit,
    size_t numBytes
)
{
#if defined(THREADED)
    uint32_t notification;
    BaseType_t status;
#endif
    bool retval = true;

    switch(io_type) {
#if defined(THREADED)
        case IO_Type::DMA:
            hw_if->transmitDMA(arrTransmit, numBytes);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_BLOCK_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                retval = false;
            }
            break;
        case IO_Type::IT:
            hw_if->transmitIT(arrTransmit, numBytes);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_TX_ISR, &notification, MAX_BLOCK_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_TX_ISR)){
                retval = false;
            }
            break;
#endif
        case IO_Type::POLL:
        default:
            retval = (hw_if->transmitPoll(arrTransmit, numBytes, TRANSFER_TIMEOUT) == HAL_OK);
            break;
    }

    if(retval != HAL_OK){
        hw_if->abortTransmit();
    }

    return retval;
}

template <class UARTInterface> bool UartDriver<UARTInterface>::receive(
    uint8_t* arrReceive,
    size_t numBytes
)
{
#if defined(THREADED)
    uint32_t notification;
    BaseType_t status;
#endif
    bool retval = true;

    switch(io_type) {
#if defined(THREADED)
        case IO_Type::DMA:
            hw_if->receiveDMA(arrReceive, numBytes);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_BLOCK_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                retval = false;
            }
            break;
        case IO_Type::IT:
            hw_if->receiveIT(arrReceive, numBytes);

            status = xTaskNotifyWait(0, NOTIFIED_FROM_RX_ISR, &notification, MAX_BLOCK_TIME);

            if(status != pdTRUE || !CHECK_NOTIFICATION(notification, NOTIFIED_FROM_RX_ISR)){
                retval = false;
            }
            break;
#endif
        case IO_Type::POLL:
        default:
            retval = (hw_if->receivePoll(arrReceive, numBytes, TRANSFER_TIMEOUT) == HAL_OK);
            break;
    }

    if(retval != HAL_OK){
        hw_if->abortReceive();
    }

    return retval;
}

} // end namespace uart_interface




/**
 * @}
 */
/* end - Header */

#endif /* __UART_DRIVER_H__ */
