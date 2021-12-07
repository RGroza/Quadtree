#include "pqueue.h"
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <assert.h>

#ifdef COUNT
#include <stdio.h>
#endif

typedef struct _pqueue_entry
{
    double priority;
    void *item;
    size_t ID;
} pqueue_entry;

struct _pqueue
{
    pqueue_entry *entries;
    size_t size;
    size_t capacity;
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

void swap_entries(pqueue_entry *entry1, pqueue_entry *entry2);
void pqueue_rebalance_heap(pqueue *q, int i);

int get_left_child(pqueue *q, int index);
int get_right_child(pqueue *q, int index);
int get_parent(pqueue *q, int index);

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

size_t pqueue_size(const pqueue *q)
{
    return q != NULL ? q->size : 0;
}

/**
 * Swaps two entries in the heap along with their struct members
 *
 * @param e1 a pointer to an entry in the pqueue
 * @param e2 a pointer to another entry in the pqueue
 */
void swap_entries(pqueue_entry *e1, pqueue_entry *e2)
{
    void *temp = e1->item;
    double pri = e1->priority;
    size_t ID = e1->ID;

    e1->priority = e2->priority;
    e2->priority = pri;
    e1->item = e2->item;
    e2->item = temp;
    e1->ID = e2->ID;
    e2->ID = ID;
}

/**
 * Returns the index of the left child
 *
 * @param q a pointer to the pqueue
 * @param i the index of the parent entry
 * @return the index of the left child
 */
int get_left_child(pqueue *q, int i)
{
    if (i >= 0 && 2 * i <= q->capacity)
    {
        return 2 * i;
    }
    return -1;
}

/**
 * Returns the index of the right child
 *
 * @param q a pointer to the pqueue
 * @param i the index of the parent entry
 * @return the index of the right child
 */
int get_right_child(pqueue *q, int i)
{
    if (i >= 0 && 2 * i + 1 <= q->capacity)
    {
        return 2 * i + 1;
    }
    return -1;
}

/**
 * Returns the index of the parent from one of the children
 *
 * @param q a pointer to the pqueue
 * @param i the index of the child
 * @return the index of the parent
 */
int get_parent(pqueue *q, int i)
{
    if (i >= 0 && i / 2 <= q->capacity)
    {
        return i / 2;
    }
    return -1;
}

bool pqueue_enqueue(pqueue *q, double pri, void *item, size_t ID)
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
        q->entries[q->size].ID = ID;
        q->size++;

        int i = q->size - 1;
        int parent_i = get_parent(q, i);

        // Keep swapping entries until the new entry has a priority that is greater than its parent
        while (i >= 0 && q->entries[i].priority < q->entries[parent_i].priority)
        {
            // Swap the new entry with its parewnt
            swap_entries(&q->entries[i], &q->entries[parent_i]);

            // Obtain the new indices for the new entry and its parent entry
            i = parent_i;
            parent_i = get_parent(q, i);
        }
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

    // Copy last item into place occupied by item with min priority
    pqueue_entry temp = q->entries[0];
    *ID = temp.ID;

    q->entries[0] = q->entries[q->size - 1];
    q->size--;

    // Rebalance the heap starting from the root entry in order to maintain an increasing order of
    // priority in the heap
    pqueue_rebalance_heap(q, 0);

    // Shrink the heap
    if (q->size < q->capacity / 4 && q->capacity / 2 >= PQUEUE_INITIAL_CAPACITY)
    {
        pqueue_embiggen(q, q->capacity / 2); // Embiggen works as shrinkify too
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

/**
 * Rebalances the heap in order to maintain increasing order of priority.
 *
 * @param q a pointer to the pqueue
 * @param i the index of parent node to start rebalancing operation from
 */
void pqueue_rebalance_heap(pqueue *q, int i)
{
    int curr = i;
    int left_i = get_left_child(q, i);
    int right_i = get_right_child(q, i);

    // Determine which child has the lower priority in the queue
    if (left_i > 0 && left_i < q->size)
    {
        if (q->entries[left_i].priority < q->entries[curr].priority)
        {
            curr = left_i;
        }
    }

    if (right_i > 0 && right_i < q->size)
    {
        if (q->entries[right_i].priority < q->entries[curr].priority)
        {
            curr = right_i;
        }
    }

    if (curr != i)
    {
        // Swap the parent and the chosen child entries
        swap_entries(&q->entries[i], &q->entries[curr]);
        pqueue_rebalance_heap(q, curr);
    }
}

void pqueue_destroy(pqueue *q, void (*f)(void *))
{
    if (q != NULL)
    {
        // Free the remaining items using the given function
        if (f != NULL)
        {
            for (size_t i = 0; i < q->size; i++)
            {
                f(q->entries[i].item);
            }
        }

        // Free the array
        free(q->entries);

#ifdef COUNT
        fprintf(stderr, "%zu enqueues, %zu dequeues\n", q->enqueue_count,
                q->dequeue_count);
#endif

        // Free the meta-data
        free(q);
    }
}