#include "ArrayList.h"

//~Constructors-----------------------------------------------
/**
 * Default constructor,
 * creates a 20 element ArrayList, of type T.
 */
template<class T>
ArrayList<T>::ArrayList() {

    array = new T[20];
    capacity = 20;
    size = 0;
}

//~Methods---------------------------------------------------
/**
 * Adds the passed in element to the end of the ArrayList.
 *
 * Runs in O(n) in worst case, where reallocate is called.
 *
 * @param element the element to add to the array.
 */
template<class T>
bool ArrayList<T>::add(T element) {

    bool value = false;

    if (element != NULL) {
        if ((size - 1) == capacity) {

            value = reallocate();
        }

        if (value) {
            array[size] = element;
            size++;
        }
    }

    return value;
}
