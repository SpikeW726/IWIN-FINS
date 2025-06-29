/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef STATICQUEUE_HPP
#define STATICQUEUE_HPP

struct UartQueueElement {
  uint8_t *data; // 数据指针
  uint16_t size; // 数据大小
};

struct I2CQueueElement {
  uint16_t devAddress;
  uint8_t *data; // 数据指针
  uint16_t size; // 数据大小
};

template <size_t N> class FixedUartQueue {
private:
  UartQueueElement buffer[N]; // 固定大小的存储空间
  size_t frontIdx = 0;    // 队首索引
  size_t rearIdx = 0;     // 队尾索引
  size_t itemCount = 0;   // 当前元素数量

public:
  FixedUartQueue() = default;

  // 入队操作
  bool enqueue(uint8_t *data, uint16_t size) {
    if (isFull()) {
      return false; // 队列已满
    }

    buffer[rearIdx] = {data, size};
    rearIdx = (rearIdx + 1) % N;
    itemCount++;
    return true;
  }

  // 出队操作
  bool dequeue() {
    if (isEmpty()) {
      return false; // 队列为空
    }

    frontIdx = (frontIdx + 1) % N;
    itemCount--;
    return true;
  }

  // 获取队首元素
  UartQueueElement& getFront(){
    return buffer[frontIdx];
  }

  // 检查队列是否为空
  bool isEmpty() { return itemCount == 0; }

  // 检查队列是否已满
  bool isFull()  { return itemCount == N; }

  // 获取当前元素数量
 size_t size() const {
  return itemCount;
 }
};


template <size_t N> class FixedI2CQueue {
private:
  I2CQueueElement buffer[N]; // 固定大小的存储空间
  size_t frontIdx = 0;    // 队首索引
  size_t rearIdx = 0;     // 队尾索引
  size_t itemCount = 0;   // 当前元素数量

public:
  FixedI2CQueue() = default;

  // 入队操作
  bool enqueue(uint16_t devAddress, uint8_t *data, uint16_t size) {
    if (isFull()) {
      return false; // 队列已满
    }

    buffer[rearIdx] = {devAddress, data, size};
    rearIdx = (rearIdx + 1) % N;
    itemCount++;
    return true;
  }

  // 出队操作
  bool dequeue() {
    if (isEmpty()) {
      return false; // 队列为空
    }

    frontIdx = (frontIdx + 1) % N;
    itemCount--;
    return true;
  }

  // 获取队首元素
  I2CQueueElement& getFront(){
    return buffer[frontIdx];
  }

  // 检查队列是否为空
  bool isEmpty() { return itemCount == 0; }

  // 检查队列是否已满
  bool isFull()  { return itemCount == N; }

  // 获取当前元素数量
  size_t size() const {
    return itemCount;
  }
};

#endif //STATICQUEUE_HPP