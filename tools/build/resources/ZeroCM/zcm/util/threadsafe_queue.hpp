#pragma once

#include "zcm/util/queue.hpp"

#include <mutex>
#include <condition_variable>
#include <atomic>

// A thread-safe C++ queue implementation designed for efficiency.
// No unneeded copies or initializations.
// Note: we wrap the Queue implementation
template<class Element>
class ThreadsafeQueue
{
    Queue<Element> queue;

    std::mutex mut;
    std::condition_variable cond;
    bool disabled = false;

  public:
    ThreadsafeQueue(size_t size) : queue(size) {}
    ~ThreadsafeQueue() {}

    size_t getCapacity()
    {
        std::unique_lock<std::mutex> lk(mut);
        return queue.getCapacity();
    }

    void setCapacity(size_t capacity)
    {
        std::unique_lock<std::mutex> lk(mut);
        return queue.setCapacity(capacity);
    }

    bool hasFreeSpace()
    {
        std::unique_lock<std::mutex> lk(mut);
        return queue.hasFreeSpace();
    }

    bool hasMessage()
    {
        std::unique_lock<std::mutex> lk(mut);
        return queue.hasMessage();
    }

    size_t numMessages()
    {
        std::unique_lock<std::mutex> lk(mut);
        return queue.numMessages();
    }

    // Wait for hasFreeSpace() and then push the new element
    // Returns true if the value was pushed, otherwise it
    // was forcibly awoken by forceWakeups()
    template<class... Args>
    bool push(Args&&... args)
    {
        std::unique_lock<std::mutex> lk(mut);
        cond.wait(lk, [&](){ return disabled || queue.hasFreeSpace(); });
        if (!queue.hasFreeSpace()) return false;

        queue.push(std::forward<Args>(args)...);
        cond.notify_all();
        return true;
    }

    // Check for hasFreeSpace() and if so, push the new element
    // Returns true if the value was pushed, returns false if no room
    template<class... Args>
    bool pushIfRoom(Args&&... args)
    {
        std::unique_lock<std::mutex> lk(mut);
        if (!queue.hasFreeSpace()) return false;

        queue.push(std::forward<Args>(args)...);
        cond.notify_all();
        return true;
    }

    // Wait for hasMessage() and then return the top element
    // Always returns a valid Element* except when is was
    // forcibly awoken by forceWakeups(). In such a case
    // nullptr is returned to the user
    Element* top()
    {
        std::unique_lock<std::mutex> lk(mut);
        cond.wait(lk, [&](){ return disabled || queue.hasMessage(); });
        if (disabled) return nullptr;

        Element& elt = queue.top();
        return &elt;
    }

    // Requires that hasMessage() == true
    void pop()
    {
        std::unique_lock<std::mutex> lk(mut);
        queue.pop();
        cond.notify_all();
    }

    // Forcefully wakes up top() and push(). top() *will not* return a message from
    // the queue, even if one exists. push() *will* push the message if there is room.
    void disable()
    {
        std::unique_lock<std::mutex> lk(mut);
        disabled = true;
        cond.notify_all();
    }

    void enable()
    {
        std::unique_lock<std::mutex> lk(mut);
        disabled = false;
    }
};
