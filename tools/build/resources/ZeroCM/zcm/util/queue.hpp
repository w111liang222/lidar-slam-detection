#pragma once

#include <utility>
#include <memory>
#include <cstring>
#include <cassert>

// A C++ queue implementation designed for efficiency.
// No unneeded copies or initializations.
// Note: Nothing about this queue is thread-safe
template<class Element>
class Queue
{
    Element* queue;
    size_t   capacity;
    size_t   front = 0;
    size_t   back = 0;

    size_t incIdx(size_t i)
    {
        // Note: one might be tempted to write '(i + 1) % capacity' here
        // But, the modulus operation is slower than possibly missing
        // a branch every once in a while. The branch is almost always
        // Not Taken

        size_t nextIdx = i + 1;
        if (nextIdx == capacity) return 0;
        return nextIdx;
    }

  public:
    Queue(size_t capacity) : capacity(capacity)
    {
        // We are avoiding initializing the structs here
        queue = (Element*) new uint8_t[capacity * sizeof(Element)];
        ZCM_ASSERT(queue);
    }

    ~Queue()
    {
        // We need to deconstruct any elements still in the queue
        while (hasMessage()) pop();
        delete[] ((uint8_t*) queue);
    }

    size_t getCapacity()
    {
        return capacity;
    }

    void setCapacity(size_t capacity)
    {
        uint8_t* newQueue = new uint8_t[capacity * sizeof(Element)];
        ZCM_ASSERT(newQueue);

        size_t newBack = 0;
        while (hasMessage() && newBack < capacity) {
            uint8_t* msg = (uint8_t*) &top();
            std::uninitialized_copy_n(msg, sizeof(Element), newQueue + newBack * sizeof(Element));
            front = incIdx(front);
            ++newBack;
        }

        delete[] ((uint8_t*) queue);
        queue = (Element*) newQueue;
        front = 0;
        back = newBack;
        this->capacity = capacity;
    }

    bool hasFreeSpace()
    {
        return front != incIdx(back);
    }

    bool hasMessage()
    {
        return front != back;
    }

    size_t numMessages()
    {
        if (back >= front) {
            return back - front;
        } else {
            return capacity - (front - back);
        }
    }

    // Requires that hasFreeSpace() == true
    template<class... Args>
    void push(Args&&... args)
    {
        assert(hasFreeSpace());

        // Initialize the Element by forwarding the parameter pack
        // directly to the constructor called via Placement New
        new (&queue[back]) Element(std::forward<Args>(args)...);

        back = incIdx(back);
    }

    // Requires that hasMessage() == true
    Element& top()
    {
        assert(hasMessage());
        return queue[front];
    }

    // Requires that hasMessage() == true
    void pop()
    {
        assert(hasMessage());
        // Manually call the destructor
        queue[front].~Element();
        front = incIdx(front);
    }

  private:
    Queue(const Queue& other) = delete;
    Queue(Queue&& other) = delete;
    Queue& operator=(const Queue& other) = delete;
    Queue& operator=(Queue&& other) = delete;
};
