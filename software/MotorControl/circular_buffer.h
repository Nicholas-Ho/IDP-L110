#ifndef CIRCULAR_BUFFER
#define CIRCULAR_BUFFER

#define ARRAY_SIZE 200

// Utility FIFO Queue implementation
// Used for Colour Detection and Derivative Control Line Following
class CircularBuffer {
    private:
        int buffer[ARRAY_SIZE] = { 0 };
        int firstIndex = 0; // Index of first element
        int lastIndex = ARRAY_SIZE-1; // Index of last element. To be overwritten.
        int bufferLength = 0; // Buffer length. When length = size, buffer is full.
        int size = -1; // Size of buffer. To be overwritten

    public:
        CircularBuffer(int bufferSize, bool initialise=false, int initValue=0);
        void add(int contents);
        int pop();
        int operator [](int i); // Override read operator to get buffer contents at index
        int resetFill(int resetValue=0);
        int getSize();
};

#endif