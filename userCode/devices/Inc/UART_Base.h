/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_UARTBASELITE_H
#define FINEMOTE_UARTBASELITE_H

#include "Usermain.h"
#include "StaticQueue.h"

#define TX_QUEUE_SIZE 20

constexpr UART_HandleTypeDef *uartHandleList[] = {nullptr, &huart1, nullptr, nullptr, nullptr, nullptr, &huart6};

template <uint8_t ID>
class UARTBaseLite{
public:
  static UARTBaseLite &GetInstance() {
    static UARTBaseLite instance;
    return instance;
  }

  void Handle() {
    TxLoader();
  }

  void Transmit(uint8_t *data, uint16_t size) {
    if(txQueue.isFull()) {
      txQueue.dequeue();
    }
    txQueue.enqueue(data,size);
    TxLoader();
  }

  void TxLoader() {
    if (!txQueue.isEmpty() && isTxFinished) {
      HAL_UART_Transmit_IT(uartHandleList[ID], txQueue.getFront().data, txQueue.getFront().size);
      isTxFinished = false;
      txQueue.dequeue();
    }
  }

  bool isTxFinished = true;

private:
  // etl::queue<std::pair<uint8_t*, uint16_t>,TX_QUEUE_SIZE> txQueue;
  FixedUartQueue<TX_QUEUE_SIZE> txQueue;
};

#endif // FINEMOTE_UARTBASELITE_H