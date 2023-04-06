#pragma once
#include <cstdlib>

// A memory pool for the UDPM fragment buffering
class MemPool
{
  public:
    MemPool();
    ~MemPool();

    char *alloc(size_t sz);
    void free(char *mem, size_t sz);

    template<class T>
    T *alloc();

    template<class T>
    void free(T *ptr);

    static void test();

  private:
    struct Block { Block *next; };
    static const size_t NUMLISTS = 13;
    Block* sizelists[NUMLISTS]; // Pow2 blocks from 2^16 to 2^28

  private:
    // Disallow copies and moves
    MemPool(const MemPool&) = delete;
    MemPool& operator=(const MemPool&) = delete;
    MemPool(MemPool&& other) = delete;
    MemPool& operator=(MemPool&& other) = delete;
};

template<class T>
T *MemPool::alloc()
{
    return (T*)this->alloc(sizeof(T));
}

template<class T>
void MemPool::free(T *ptr)
{
    this->free((char*)ptr, sizeof(*ptr));
}
