#include <Arduino.h>
//#include <iostream>

#include "circular_buffer.h"

CircularBuffer::CircularBuffer(int bufferSize, bool initialise=false, int initValue=0) {
    // bufferSize: Size of buffer. Cannot be more than array size
    // initialise: If true, initialise a full buffer with initValue
    // initValue: Value to initialise with. Does nothing if initialise=false

    // Ensure that size is not larger than array
    size = (bufferSize <= ARRAY_SIZE) ? bufferSize : ARRAY_SIZE;

    if(initValue != 0) {
      for(int i=0; i < (ARRAY_SIZE); i++) {
          buffer[i] = initValue;
      }
    }

    if(initialise == true) {
        lastIndex = size-1;
        bufferLength = size;
    }
}

void CircularBuffer::add(int contents) {
    if(bufferLength == 0) {
        lastIndex = firstIndex;
        buffer[lastIndex] = contents;
        bufferLength++;
        return;
    }

    lastIndex = (lastIndex + 1) % ARRAY_SIZE;

    if(bufferLength < size) {
        bufferLength++;
        buffer[lastIndex] = contents;
    } else {
        Serial.println("Queue full, contents not added.");
        // std::cout << "Queue full, contents not added.\n";
    }
}

int CircularBuffer::pop() {
    if(bufferLength > 0) {
        int output = buffer[firstIndex];
        firstIndex = (firstIndex + 1) % ARRAY_SIZE;
        bufferLength--;

        return output;
    } else {
        Serial.println("Queue empty, nothing to pop.");
        // std::cout << "Queue empty, nothing to pop\n";
        return -1;
    }
}

// Reset and fill entire buffer with same value
int CircularBuffer::resetFill(int resetValue) {
  for(int i=0; i < (ARRAY_SIZE); i++) {
      buffer[i] = resetValue;
  }
  firstIndex = 0;
  lastIndex = size-1;
  bufferLength = size;
  return 0;
}

// Override read operator to get buffer at index
int CircularBuffer::operator [](int i) {
    if(i < bufferLength) {
        return buffer[(firstIndex+i)%ARRAY_SIZE];
    } else {
        Serial.println("Index out of range.");
        // std::cout << "Index out of range.";
        return -1;
    }
}

// // Testing
// int main() {
//     CircularBuffer buffer(5);
//     buffer.add(1);
//     buffer.add(2);
//     buffer.add(4);
//     std::cout << buffer[0] << " " << buffer[1] << " " << buffer[2] << "\n";
//     for(int i=0; i<4; i++) {
//          std::cout<< buffer.pop() << "\n";
//     }
//     for(int i=0; i<10; i++) {
//         buffer.add(i);
//     }
//     std::cout << buffer[0] << " " << buffer[1] << " " << buffer[2] << " " << buffer[3] << " " << buffer[4] << "\n";
//     for(int i=0; i<5; i++) {
//         buffer.pop();
//     }
//     for(int i=0; i<5; i++) {
//         buffer.add(i*3);
//     }
//     std::cout << buffer[0] << " " << buffer[1] << " " << buffer[2] << " " << buffer[3] << " " << buffer[4] << "\n";
//     return 0;
// };