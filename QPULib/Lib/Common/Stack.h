#ifndef _STACK_H_
#define _STACK_H_

#include <cstdlib>
#include <cassert>

template <class T> class StackItem
{
public:
    T* head;
    StackItem<T>* tail;
};

template <class T> class Stack
{
public:
    unsigned int size;
    StackItem<T>* topItem;

    // Constructor
    Stack()
    {
        topItem = NULL;
        size    = 0;
    }

    // Push
    void push(T* x)
    {
        StackItem<T>* oldTop = topItem;
        topItem       = new StackItem<T>;
        topItem->head = x;
        topItem->tail = oldTop;
        size++;
    }

    // Pop
    void pop()
    {
        assert(size > 0);
        StackItem<T>* oldTop = topItem;
        topItem = topItem->tail;
        delete oldTop;
        size--;
    }

    // Top
    T* top()
    {
        assert(size > 0);
        return topItem->head;
    }

    // Replace the top element
    void replace(T* x)
    {
        topItem->head = x;
    }

    // Clear the stack
    void clear()
    {
        StackItem<T>* p;
        for (unsigned int i = 0; i < size; i++)
        {
            p = topItem->tail;
            delete topItem;
            topItem = p;
        }
        size = 0;
    }

    // Obtain Nth element from the top
    T* index(unsigned int n)
    {
        assert(n < size);
        StackItem<T>* p = topItem;
        for (int i = 0; i < n; i++)
            p = p->tail;
        return p->head;
    }

    // Destructor
    ~Stack()
    {
        clear();
    }
};

#endif
