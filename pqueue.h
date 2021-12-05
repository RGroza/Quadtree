#ifndef __PQUEUE_H__
#define __PQUEUE_H__

#include <stddef.h>
#include <stdbool.h>

/**
 * A priority queue for generic items with double values as priorities.
 */
typedef struct _pqueue pqueue;

/**
 * Creates an empty priority queue.  Returns NULL if there was a problem
 * allocating memory for the queue.  It is the caller's responsibility
 * to eventually destroy the returned queue if it is non-NULL.
 *
 * @return a pointer to the new queue, or NULL
 */
pqueue *pqueue_create();

/**
 * Returns the number of items on the given priority queue.
 *
 * @param a pointer to a priority queue, non-NULL
 * @return the number of items on that queue
 */
size_t pqueue_size(const pqueue *q);

/**
 * Adds the given item to the given queue with the given priority.
 * If there is a memory allocation error during the operation then
 * the return value is false and there is no change to the queue.
 * Otherwise, the return value is true.  The caller retains ownership
 * of the item.
 *
 * @param q a pointer to a priority queue, non-NULL
 * @param pri a number
 * @param item a pointer
 */
bool pqueue_enqueue(pqueue *q, double pri, void *item);

/**
 * Removes the item with the lowest priority from the given queue and
 * returns it.  The priority of the removed item is written to the
 * double pointed to by the last argument if it is non-NULL.
 *
 * @param q a pointer to a non-empty priority queue, non-NULL
 * @param pri a pointer to a double, or NULL
 * @return a pointer to the removed item
 */
void *pqueue_dequeue(pqueue *q, double *pri, size_t *ID);

/**
 * Destroys the given priority queue.  Al remaining items are passed
 * to the given function if it is not NULL.  There is no effect if the
 * pointer to the queue is NULL.
 *
 * @param q q pointer to a priority queue
 * @param f a pointer to a function that frees the items, or NULL
 */
void pqueue_destroy(pqueue *q, void (*f)(void *));

#endif
