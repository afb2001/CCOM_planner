
#include <cstring>
#include <iostream>
template <typename T>
PQueue<T>::PQueue() // default constructor, construct an empty heap
{
        _size = 0;
        _array = new T[capSize];
}

template <typename T>
PQueue<T>::PQueue(unsigned int (*c)(T &t1, T &t2)) // construct a heap from an array of elements;
{
        _size = 0;
        compare = c;
        _array = new T[capSize];
}

template <typename T>
PQueue<T>::~PQueue()
{
        delete[] _array;
}

template <typename T>
PQueue<T>::PQueue(const PQueue &rhs)
{
        capSize = rhs.capSize;
        _size = rhs._size;
        _array = new T[capSize];
        for (int i = 0; i < _size; i++)
                _array[i] = rhs._array[i];
        compare = rhs.compare;
}

template <typename T>
PQueue<T>& PQueue<T>::operator=(const PQueue &rhs)
{
        if (this == &rhs)
                return *this;
        delete[] _array;
        capSize = rhs.capSize;
        _size = rhs._size;
        _array = new T[capSize];
        for (int i = 0; i < _size; i++)
                _array[i] = rhs._array[i];
        compare = rhs.compare;
        return *this;
}

template <typename T>
void PQueue<T>::push(T a) // insert an item; duplicates are allowed.
{
        index = _size++;
        parentindex = (index - 1) >> 1;
        _array[index] = a;
        while (index != 0 && compare(_array[index], _array[parentindex]))
        {
                temp = _array[parentindex];
                _array[parentindex] = _array[index];
                _array[index] = temp;
                index = parentindex;
                parentindex = (index - 1) >> 1;
        }

        if (capSize == _size)
        {
                T *temparray = _array;
                _array = new T[capSize << 1];
                memcpy(_array, temparray, sizeof(T) * capSize);
                delete[] temparray;
                capSize = capSize << 1;
        }
}

template <typename T>
T PQueue<T>::top() const // return the smallest item from the queue
{
        return _array[0];
}

template <typename T>
T PQueue<T>::pop() // remove the smallest item from the queue
{
        if (_size == 0)
        {
                std::cerr << "ERROR POP\n"
                          << std::endl;
                exit(-1);
        }

        tempreturn = _array[0];
        _array[0] = _array[--_size];
        moveDown(0);
        return tempreturn;
}

template <typename T>
void PQueue<T>::remove() // remove the smallest item from the queue
{
        if (_size == 0)
        {
                std::cerr << "ERROR REMOVE\n"
                          << std::endl;
                exit(-1);
        }

        _array[0] = _array[--_size];
        moveDown(0);
}

template <typename T>
void PQueue<T>::clear() // test if the priority queue is logically empty
{
        _size = 0;
}

template <typename T>
bool PQueue<T>::empty() const // test if the priority queue is logically empty
{
        return _size == 0;
}

template <typename T>
void PQueue<T>::reBuild() // rebuild array
{
        buildHeap();
}

template <typename T>
T &PQueue<T>::operator[](unsigned int index)
{
        return _array[index];
}

template <typename T>
int PQueue<T>::size() const // return queue size
{
        return _size;
}
template <typename T>
void PQueue<T>::buildHeap() // linear heap construction
{
        //printf("Hello1 %d\n", _array[1]);
        for (int i = _size >> 1; i >= 0; i--)
                moveDown(i);
        //printf("Hello2 %d\n", _array[1]);
}
template <typename T>
void PQueue<T>::moveDown(int i) // move down element at given index
{
        l = (i << 1) + 1;
        index = i;
        if (l < _size)
        {
                if (compare(_array[l], _array[index]))
                        index = l;
                r = l + 1;
                if (r < _size && compare(_array[r], _array[index]))
                        index = r;
        }

        if (index != i)
        {
                temp = _array[i];
                _array[i] = _array[index];
                _array[index] = temp;
                moveDown(index);
        }
}

template <typename T>
void PQueue<T>::moveUP(int i) // move down element at given index
{
        index = i;
        parentindex = (index - 1) >> 1;
        while (index != 0 && compare(_array[index], _array[parentindex]))
        {
                temp = _array[parentindex];
                _array[parentindex] = _array[index];
                _array[index] = temp;
                index = parentindex;
                parentindex = (index - 1) >> 1;
        }
}

template <typename T>
void PQueue<T>::setUpCompare(unsigned int (*c)(T &t1, T &t2))
{
        compare = c;
        buildHeap();
}
