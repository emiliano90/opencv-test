#ifndef ARRAYLIST_H_
#define ARRAYLIST_H_

#include <string>

template<class T>
class ArrayList {

    private:
         //Data fields-------------------//
        T *array;
        int size;
        int capacity;
        bool sorted;

        //Method s-----------------------//
        void reallocate();
        void reallocate(int newSize);

        T* mergeSort(T* array, int arraySize);

    public:
        //Constructors------------------//
        ArrayList();
        ArrayList(int theSize);

        //Methods-----------------------//
        //Operations
        bool add(T element);
        bool add(T element, int index);
        bool add(ArrayList<T> list);
        bool add(ArrayList<T> list, int index);
        std:string toString();
};

#endif /* ARRAYLIST_H_ */