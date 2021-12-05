#include "pointset.h"
#include "point2d.h"
#include "pqueue.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

struct _pointset
{
    struct _pointnode *root;
    size_t size;
};

typedef struct _pointnode
{
    point2d *pt;
    struct _pointnode *NW;
    struct _pointnode *SW;
    struct _pointnode *SE;
    struct _pointnode *NE;
    point2d *region_ll;
    point2d *region_ur;
} pointnode;

pointset *pointset_create(const point2d *pts, size_t n);
void pointset_recursive(pointnode *root, const point2d *pts, size_t n);
void pointset_set_region(pointnode *node, point2d ll, point2d ur);
size_t pointset_size(const pointset *t);
bool pointset_add(pointset *t, const point2d *pt);
bool pointset_contains(const pointset *t, const point2d *pt);
void pointset_nearest_neighbor(const pointset *t, const point2d *pt,
                               point2d *neighbor, double *d);
point2d *pointset_k_nearest(const pointset *t, const point2d *pt, size_t k);
void pointset_for_each(const pointset *t, void (*f)(const point2d *, void *),
                       void *arg);
void pointset_destroy(pointset *t);
void pointnode_delete(pointnode *root);
int compare_points(const void *p1, const void *p2);
pointnode *find_path(pointnode *root, const point2d *pt);
void recursive_search(const pointnode *root, void (*f)(const point2d *, void *),
                      void *arg);
void node_nearest_neighbor(pointnode *root, const point2d *pt,
                           point2d *neighbor, double *d);
void check_distance(pointnode *node, const point2d *pt, point2d *neighbor,
                    double *d);
bool node_is_end(pointnode *node);

pointset *pointset_create(const point2d *pts, size_t n)
{
    pointset *result = calloc(1, sizeof(pointset));
    if (result == NULL)
    {
        return NULL;
    }

    result->root = calloc(1, sizeof(pointnode));
    if (result->root == NULL)
    {
        return NULL;
    }

    result->root->NW = NULL;
    result->root->SW = NULL;
    result->root->SE = NULL;
    result->root->NE = NULL;
    result->root->pt = NULL;
    result->root->region_ll = NULL;
    result->root->region_ur = NULL;

    result->size = n;
    if (pts == NULL || n == 0)
    {
        return result;
    }

    point2d *pts_cpy = malloc(sizeof(point2d) * n);
    memcpy(pts_cpy, pts, sizeof(point2d) * n);

    qsort(pts_cpy, n, sizeof(point2d), compare_points);

    result->root->pt = malloc(sizeof(point2d));
    result->root->pt->x = pts_cpy[n / 2].x;
    result->root->pt->y = pts_cpy[n / 2].y;

    pointset_set_region(result->root, (point2d){-INFINITY, -INFINITY}, (point2d){INFINITY, INFINITY});

    pointset_recursive(result->root, pts_cpy, n);

    free(pts_cpy);

    return result;
}

void pointset_recursive(pointnode *root, const point2d *pts, size_t n)
{
    point2d *NW_list = malloc(sizeof(point2d) * n / 2);
    size_t NW_size = 0;
    point2d *SW_list = malloc(sizeof(point2d) * n / 2);
    size_t SW_size = 0;
    for (int i = 0; i < n / 2; i++)
    {
        if (pts[i].y > root->pt->y)
        {
            NW_list[NW_size] = pts[i];
            NW_size++;
        }
        else
        {
            SW_list[SW_size] = pts[i];
            SW_size++;
        }
    }

    if (NW_size > 0)
    {
        pointnode *NW_child = malloc(sizeof(pointnode));
        NW_child->pt = malloc(sizeof(point2d));
        NW_child->pt->x = NW_list[NW_size / 2].x;
        NW_child->pt->y = NW_list[NW_size / 2].y;
        NW_child->NW = NULL;
        NW_child->SW = NULL;
        NW_child->SE = NULL;
        NW_child->NE = NULL;

        pointset_set_region(NW_child, (point2d){root->region_ll->x, root->pt->y},
                            (point2d){root->pt->x, root->region_ur->y});

        root->NW = NW_child;

        if (NW_size > 1)
        {
            pointset_recursive(NW_child, NW_list, NW_size);
        }
    }
    free(NW_list);

    if (SW_size > 0)
    {
        pointnode *SW_child = malloc(sizeof(pointnode));
        SW_child->pt = malloc(sizeof(point2d));
        SW_child->pt->x = SW_list[SW_size / 2].x;
        SW_child->pt->y = SW_list[SW_size / 2].y;
        SW_child->NW = NULL;
        SW_child->SW = NULL;
        SW_child->SE = NULL;
        SW_child->NE = NULL;

        pointset_set_region(SW_child, *root->region_ll, *root->pt);

        root->SW = SW_child;

        if (SW_size > 1)
        {
            pointset_recursive(SW_child, SW_list, SW_size);
        }
    }
    free(SW_list);

    point2d *NE_list = malloc(sizeof(point2d) * n / 2);
    size_t NE_size = 0;
    point2d *SE_list = malloc(sizeof(point2d) * n / 2);
    size_t SE_size = 0;
    for (int i = n / 2 + 1; i < n; i++)
    {
        if (pts[i].y > root->pt->y)
        {
            NE_list[NE_size] = pts[i];
            NE_size++;
        }
        else
        {
            SE_list[SE_size] = pts[i];
            SE_size++;
        }
    }

    if (NE_size > 0)
    {
        pointnode *NE_child = malloc(sizeof(pointnode));
        NE_child->pt = malloc(sizeof(point2d));
        NE_child->pt->x = NE_list[NE_size / 2].x;
        NE_child->pt->y = NE_list[NE_size / 2].y;
        NE_child->NW = NULL;
        NE_child->SW = NULL;
        NE_child->SE = NULL;
        NE_child->NE = NULL;

        pointset_set_region(NE_child, *root->pt, *root->region_ur);

        root->NE = NE_child;

        if (NE_size > 1)
        {
            pointset_recursive(NE_child, NE_list, NE_size);
        }
    }
    free(NE_list);

    if (SE_size > 0)
    {
        pointnode *SE_child = malloc(sizeof(pointnode));
        SE_child->pt = malloc(sizeof(point2d));
        SE_child->pt->x = SE_list[SE_size / 2].x;
        SE_child->pt->y = SE_list[SE_size / 2].y;
        SE_child->NW = NULL;
        SE_child->SW = NULL;
        SE_child->SE = NULL;
        SE_child->NE = NULL;

        pointset_set_region(SE_child, *root->pt, (point2d){root->region_ur->x, root->region_ll->y});

        root->SE = SE_child;

        if (SE_size > 1)
        {
            pointset_recursive(SE_child, SE_list, SE_size);
        }
    }
    free(SE_list);
}

void pointset_set_region(pointnode *node, point2d ll, point2d ur)
{
    if (node != NULL)
    {
        node->region_ll = malloc(sizeof(point2d));
        node->region_ll->x = ll.x;
        node->region_ll->y = ll.y;

        node->region_ur = malloc(sizeof(point2d));
        node->region_ur->x = ur.x;
        node->region_ur->y = ur.y;
    }
}

size_t pointset_size(const pointset *t) { return t->size; }

bool pointset_add(pointset *t, const point2d *pt)
{
    if (t->root->pt == NULL)
    {
        t->root->pt = malloc(sizeof(point2d));
        if (t->root->pt == NULL)
        {
            return false;
        }

        t->root->pt->x = pt->x;
        t->root->pt->y = pt->y;

        pointset_set_region(t->root, (point2d){-INFINITY, -INFINITY}, (point2d){INFINITY, INFINITY});

        t->size++;

        return true;
    }

    if (pointset_contains(t, pt))
    {
        return false;
    }

    pointnode *curr_node = t->root;
    pointnode *prev_node;
    while (curr_node != NULL)
    {
        prev_node = curr_node;
        curr_node = find_path(curr_node, pt);
    }

    pointnode *new_node;
    if (pt->x < prev_node->pt->x)
    {
        if (pt->y > prev_node->pt->y)
        {
            prev_node->NW = malloc(sizeof(pointnode));
            new_node = prev_node->NW;

            pointset_set_region(new_node, (point2d){prev_node->region_ll->x, prev_node->pt->y},
                                (point2d){prev_node->pt->x, prev_node->region_ur->y});
        }
        else
        {
            prev_node->SW = malloc(sizeof(pointnode));
            new_node = prev_node->SW;

            pointset_set_region(new_node, *prev_node->region_ll, *prev_node->pt);
        }
    }
    else
    {
        if (pt->y > prev_node->pt->y)
        {
            prev_node->NE = malloc(sizeof(pointnode));
            new_node = prev_node->NE;

            pointset_set_region(new_node, *prev_node->pt, *prev_node->region_ur);
        }
        else
        {
            prev_node->SE = malloc(sizeof(pointnode));
            new_node = prev_node->SE;

            pointset_set_region(new_node, *prev_node->pt,
                                (point2d){prev_node->region_ur->x, prev_node->region_ll->y});
        }
    }

    if (new_node == NULL)
    {
        return false;
    }

    new_node->pt = malloc(sizeof(point2d));
    new_node->pt->x = pt->x;
    new_node->pt->y = pt->y;
    new_node->NW = NULL;
    new_node->SW = NULL;
    new_node->SE = NULL;
    new_node->NE = NULL;

    t->size++;

    return true;
}

bool pointset_contains(const pointset *t, const point2d *pt)
{
    if (t->root->pt == NULL)
    {
        return false;
    }

    pointnode *curr_node = t->root;
    while (curr_node->pt->x != pt->x || curr_node->pt->y != pt->y)
    {
        curr_node = find_path(curr_node, pt);
        if (curr_node == NULL)
        {
            return false;
        }
    }

    return true;
}

int compare_points(const void *p1, const void *p2)
{
    point2d pt1 = *(point2d *)p1;
    point2d pt2 = *(point2d *)p2;

    if (pt1.x > pt2.x)
    {
        return 1;
    }
    else if (pt1.x < pt2.x)
    {
        return -1;
    }
    else
    {
        if (pt1.y > pt2.y)
        {
            return 1;
        }
        else if (pt1.y < pt2.y)
        {
            return -1;
        }
        else
        {
            return 0;
        }
    }
}

pointnode *find_path(pointnode *root, const point2d *pt)
{
    if (pt->x < root->pt->x)
    {
        if (pt->y > root->pt->y)
        {
            return root->NW;
        }
        else
        {
            return root->SW;
        }
    }
    else
    {
        if (pt->y > root->pt->y)
        {
            return root->NE;
        }
        else
        {
            return root->SE;
        }
    }
}

void pointset_destroy(pointset *t)
{
    pointnode_delete(t->root);
    free(t);
}

void pointnode_delete(pointnode *root)
{
    if (root->NW != NULL)
    {
        pointnode_delete(root->NW);
    }
    if (root->SW != NULL)
    {
        pointnode_delete(root->SW);
    }
    if (root->SE != NULL)
    {
        pointnode_delete(root->SE);
    }
    if (root->NE != NULL)
    {
        pointnode_delete(root->NE);
    }
    free(root->pt);
    free(root->region_ll);
    free(root->region_ur);
    free(root);
}

void pointset_for_each(const pointset *t, void (*f)(const point2d *, void *),
                       void *arg)
{
    recursive_search(t->root, f, arg);
}

void recursive_search(const pointnode *root, void (*f)(const point2d *, void *),
                      void *arg)
{
    if (root->NW != NULL)
    {
        recursive_search(root->NW, f, arg);
    }
    if (root->SW != NULL)
    {
        recursive_search(root->SW, f, arg);
    }
    if (root->SE != NULL)
    {
        recursive_search(root->SE, f, arg);
    }
    if (root->NE != NULL)
    {
        recursive_search(root->NE, f, arg);
    }
    if (root->pt != NULL)
    {
        f(root->pt, arg);
    }
}

void pointset_nearest_neighbor(const pointset *t, const point2d *pt,
                               point2d *neighbor, double *d)
{
    if (t->size < 1)
    {
        *d = INFINITY;
    }
    if (t->root != NULL && t->root->pt != NULL)
    {
        *d = point2d_distance(pt, t->root->pt);
        node_nearest_neighbor(t->root, pt, neighbor, d);
    }
}

void node_nearest_neighbor(pointnode *root, const point2d *pt,
                           point2d *neighbor, double *d)
{
    if (root->NW != NULL)
    {
        if (node_is_end(root->NW) && root->NW->pt != NULL)
        {
            check_distance(root->NW, pt, neighbor, d);
        }
        else
        {
            point2d *NW_ll = malloc(sizeof(point2d));
            NW_ll->x = 0;
            NW_ll->y = root->pt->y;
            point2d *NW_ur = malloc(sizeof(point2d));
            NW_ur->x = root->pt->x;
            NW_ur->y = INFINITY;
            if (point2d_distance_to_rectangle(pt, NW_ll, NW_ur) < *d)
            {
                node_nearest_neighbor(root->NW, pt, neighbor, d);
                check_distance(root, pt, neighbor, d);
            }
            free(NW_ll);
            free(NW_ur);
        }
    }

    if (root->SW != NULL)
    {
        if (node_is_end(root->SW) && root->SW->pt != NULL)
        {
            check_distance(root->SW, pt, neighbor, d);
        }
        else
        {
            point2d *SW_ll = malloc(sizeof(point2d));
            SW_ll->x = 0;
            SW_ll->y = 0;
            point2d *SW_ur = malloc(sizeof(point2d));
            SW_ur->x = root->pt->x;
            SW_ur->y = root->pt->y;
            if (point2d_distance_to_rectangle(pt, SW_ll, SW_ur) < *d)
            {
                node_nearest_neighbor(root->SW, pt, neighbor, d);
                check_distance(root, pt, neighbor, d);
            }
            free(SW_ll);
            free(SW_ur);
        }
    }

    if (root->SE != NULL)
    {
        if (node_is_end(root->SE) && root->SE->pt != NULL)
        {
            check_distance(root->SE, pt, neighbor, d);
        }
        else
        {
            point2d *SE_ll = malloc(sizeof(point2d));
            SE_ll->x = root->pt->x;
            SE_ll->y = 0;
            point2d *SE_ur = malloc(sizeof(point2d));
            SE_ur->x = INFINITY;
            SE_ur->y = root->pt->y;
            if (point2d_distance_to_rectangle(pt, SE_ll, SE_ur) < *d)
            {
                node_nearest_neighbor(root->SE, pt, neighbor, d);
                check_distance(root, pt, neighbor, d);
            }
            free(SE_ll);
            free(SE_ur);
        }
    }

    if (root->NE != NULL)
    {
        if (node_is_end(root->NE) && root->NE->pt != NULL)
        {
            check_distance(root->NE, pt, neighbor, d);
        }
        else
        {
            point2d *NE_ll = malloc(sizeof(point2d));
            NE_ll->x = root->pt->x;
            NE_ll->y = root->pt->y;
            point2d *NE_ur = malloc(sizeof(point2d));
            NE_ur->x = INFINITY;
            NE_ur->y = INFINITY;
            if (point2d_distance_to_rectangle(pt, NE_ll, NE_ur) < *d)
            {
                node_nearest_neighbor(root->NE, pt, neighbor, d);
                check_distance(root, pt, neighbor, d);
            }
            free(NE_ll);
            free(NE_ur);
        }
    }
}

void check_distance(pointnode *node, const point2d *pt, point2d *neighbor,
                    double *d)
{
    double dist = point2d_distance(pt, node->pt);
    if (dist < *d)
    {
        *d = dist;
        neighbor->x = node->pt->x;
        neighbor->y = node->pt->y;
    }
}

bool node_is_end(pointnode *node)
{
    if (node->NW == NULL && node->SW == NULL && node->SE == NULL &&
        node->NE == NULL)
    {
        return true;
    }
    return false;
}

point2d *pointset_k_nearest(const pointset *t, const point2d *pt, size_t k)
{
    pqueue *q = pqueue_create();
    point2d nearest[k];
    size_t found = 0;

    pqueue_enqueue(q, 0, t->root, 1);
    while (pqueue_size(q) != 0 && found < k)
    {
        size_t *item_ID = malloc(sizeof(size_t));
        void *item = pqueue_dequeue(q, NULL, item_ID);
        if (item_ID == 0)
        {
            nearest[found] = *(point2d *)item;
            found++;
        }
        else
        {
            pointnode *node = (pointnode *)item;
            pqueue_enqueue(q, point2d_distance(pt, node->pt), node->pt, 0);

            if (node->NW != NULL)
            {
                pqueue_enqueue(q, point2d_distance_to_rectangle(pt, node->NW->region_ll, node->NW->region_ur), node->NW, 1);
            }
            if (node->SW != NULL)
            {
                pqueue_enqueue(q, point2d_distance_to_rectangle(pt, node->SW->region_ll, node->SW->region_ur), node->SW, 1);
            }
            if (node->SE != NULL)
            {
                pqueue_enqueue(q, point2d_distance_to_rectangle(pt, node->SE->region_ll, node->SE->region_ur), node->SE, 1);
            }
            if (node->NE != NULL)
            {
                pqueue_enqueue(q, point2d_distance_to_rectangle(pt, node->NE->region_ll, node->NE->region_ur), node->NE, 1);
            }
        }
    }
}