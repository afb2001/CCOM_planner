#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H



// Minimal Priority Queue implemented with a binary heap
// Stores item of type T
template <class T>
class PQueue
{
    //typedef unsigned int (*comparefunction)(T &t1, T &t2);
  public:
    PQueue();
    PQueue(unsigned int (*compare)(T &t1, T &t2)); // construct a heap from an array of elements
    ~PQueue();
    PQueue(const PQueue &rhs);
    PQueue<T>& operator=(const PQueue &rhs);
    void push(T);   // insert an item; duplicates are allowed.
    T top() const;        // return the smallest item from the queue
    T pop();        // return & remove the smallest item from the queue
    void remove();  // remove the smallest item from the queue
    void reBuild(); //rebuild the array;
    T &operator[](unsigned int);
    void clear();
    bool empty() const; // test if the priority queue is logically empty
    int size() const;     // return queue size
    void setUpCompare(unsigned int (*c)(T &t1, T &t2));
    void moveUP(int);
  private:
    unsigned int capSize = 2000;
    unsigned int index, parentindex,l,r;
    T temp, tempreturn;
    int _size; // number of queue elements
    T *_array; // the heap array, items are stoed starting at index 1
    unsigned int (*compare)(T &t1, T &t2);
    void buildHeap();   // linear heap construction
    void moveDown(int); // move down element at given index
    
};

#include "PQueue.cpp"

#endif
