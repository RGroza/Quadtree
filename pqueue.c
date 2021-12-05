#include "pqueue.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>

// leave this commented out
//#define COUNT

#ifdef COUNT
#include <stdio.h>
#endif

typedef struct _pqueue_entry
{
  double priority;
  void *item;
} pqueue_entry;

struct _pqueue
{
  pqueue_entry *entries;
  size_t size;
  size_t capacity;
#ifdef COUNT
  // these were for the nearest neighbor running time experiments
  size_t enqueue_count;
  size_t dequeue_count;
#endif
};

#define PQUEUE_INITIAL_CAPACITY (10)

/**
 * Sets the capacity of the array holding the entries in the given queue.
 * There is no effect on the queue if the allocation fails.  Note that,
 * despite the name, this works for both enlarging and shrinking the array.
 *
 * @param q a pointer to a priority queue, non-NULL
 * @param new_capacity a positive integer
 */
static void pqueue_embiggen(pqueue *q, size_t new_capacity);

pqueue *pqueue_create()
{
  pqueue *q = malloc(sizeof(*q));
  if (q != NULL)
  {
    q->entries = malloc(sizeof(*q->entries) * PQUEUE_INITIAL_CAPACITY);
    if (q->entries == NULL)
    {
      free(q);
      return NULL;
    }
    q->size = 0;
    q->capacity = PQUEUE_INITIAL_CAPACITY;
#ifdef COUNT
    q->enqueue_count = 0;
    q->dequeue_count = 0;
#endif
  }
  return q;
}

size_t pqueue_size(const pqueue *q) { return q != NULL ? q->size : 0; }

bool pqueue_enqueue(pqueue *q, double pri, void *item)
{
  if (q == NULL || isnan(pri))
  {
    return false;
  }

  if (q->size == q->capacity)
  {
    pqueue_embiggen(q, q->capacity * 2);
  }

  if (q->size < q->capacity)
  {
    q->entries[q->size].priority = pri;
    q->entries[q->size].item = item;
    q->size++;
#ifdef COUNT
    q->enqueue_count++;
#endif
    return true;
  }
  else
  {
    return false;
  }
}

void pqueue_embiggen(pqueue *q, size_t new_capacity)
{
  pqueue_entry *bigger = realloc(q->entries, new_capacity * sizeof(*bigger));
  if (bigger != NULL)
  {
    q->entries = bigger;
    q->capacity = new_capacity;
  }
}

void *pqueue_dequeue(pqueue *q, double *pri, size_t *ID)
{
  if (q == NULL || q->size == 0)
  {
    return NULL;
  }

  // find the lowest priority
  double min = q->entries[0].priority;
  size_t min_index = 0;
  for (size_t i = 1; i < q->size; i++)
  {
    if (q->entries[i].priority < min)
    {
      min = q->entries[i].priority;
      min_index = i;
    }
  }

  // copy last item into place occupied by item with min priority
  pqueue_entry temp = q->entries[min_index];
  q->entries[min_index] = q->entries[q->size - 1];
  q->size--;

  // shrink the array
  if (q->size < q->capacity / 4 && q->capacity / 2 >= PQUEUE_INITIAL_CAPACITY)
  {
    pqueue_embiggen(q, q->capacity / 2); // embiggen works as shrinkify too
  }

  if (pri != NULL)
  {
    *pri = temp.priority;
  }

#ifdef COUNT
  q->dequeue_count++;
#endif

  return temp.item;
}

void pqueue_destroy(pqueue *q, void (*f)(void *))
{
  if (q != NULL)
  {
    // free the remaining items using the given function
    if (f != NULL)
    {
      for (size_t i = 0; i < q->size; i++)
      {
        f(q->entries[i].item);
      }
    }

    // free the array
    free(q->entries);

#ifdef COUNT
    fprintf(stderr, "%zu enqueues, %zu dequeues\n", q->enqueue_count,
            q->dequeue_count);
#endif

    // free the meta-data
    free(q);
  }
}
