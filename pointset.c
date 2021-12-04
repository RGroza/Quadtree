#include "pointset.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "point2d.h"

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
} pointnode;

pointset *pointset_create(const point2d *pts, size_t n);
void pointset_recursive(pointnode *root, const point2d *pts, size_t n);
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
bool node_is_end(pointnode *node);

pointset *pointset_create(const point2d *pts, size_t n)
{
    pointset *result = calloc(1, sizeof(pointset));
    result->root = calloc(1, sizeof(pointnode));

    if (result == NULL || result->root == NULL)
    {
        return NULL;
    }

    result->size = n;
    if (pts == NULL && n == 0)
    {
        return result;
    }

    point2d *pts_cpy = malloc(sizeof(point2d) * n);
    memcpy(pts_cpy, pts, sizeof(point2d) * n);

    qsort(pts_cpy, n, sizeof(point2d), compare_points);

    result->root->pt = malloc(sizeof(point2d));
    result->root->pt->x = pts_cpy[n / 2].x;
    result->root->pt->y = pts_cpy[n / 2].y;

    pointset_recursive(result->root, pts_cpy, n);

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

        root->SE = SE_child;

        if (SE_size > 1)
        {
            pointset_recursive(SE_child, SE_list, SE_size);
        }
    }
    free(SE_list);
}

size_t pointset_size(const pointset *t) { return t->size; }

bool pointset_add(pointset *t, const point2d *pt)
{
    if (pointset_contains(t, pt))
    {
        return false;
    }

    if (t->root->pt == NULL)
    {
        t->root->pt = malloc(sizeof(point2d));
        if (t->root->pt == NULL)
        {
            return false;
        }

        t->root->pt->x = pt->x;
        t->root->pt->y = pt->y;

        return true;
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
        }
        else
        {
            prev_node->SW = malloc(sizeof(pointnode));
            new_node = prev_node->SW;
        }
    }
    else
    {
        if (pt->y > prev_node->pt->y)
        {
            prev_node->NE = malloc(sizeof(pointnode));
            new_node = prev_node->NE;
        }
        else
        {
            prev_node->SE = malloc(sizeof(pointnode));
            new_node = prev_node->SE;
        }
    }

    if (new_node == NULL)
    {
        return false;
    }

    new_node->pt = malloc(sizeof(point2d));
    new_node->pt->x = pt->x;
    new_node->pt->y = pt->y;

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
            double dist = point2d_distance(pt, root->NW->pt);
            if (dist < *d)
            {
                *d = dist;
                neighbor->x = root->NW->pt->x;
                neighbor->y = root->NW->pt->y;
            }
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
            }
            free(NW_ll);
            free(NW_ur);
        }
    }

    if (root->SW != NULL)
    {
        if (node_is_end(root->SW) && root->SW->pt != NULL)
        {
            double dist = point2d_distance(pt, root->SW->pt);
            if (dist < *d)
            {
                *d = dist;
                neighbor->x = root->SW->pt->x;
                neighbor->y = root->SW->pt->y;
            }
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
            }
            free(SW_ll);
            free(SW_ur);
        }
    }

    if (root->SE != NULL)
    {
        if (node_is_end(root->SE) && root->SE->pt != NULL)
        {
            double dist = point2d_distance(pt, root->SE->pt);
            if (dist < *d)
            {
                *d = dist;
                neighbor->x = root->SE->pt->x;
                neighbor->y = root->SE->pt->y;
            }
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
            }
            free(SE_ll);
            free(SE_ur);
        }
    }

    if (root->NE != NULL)
    {
        if (node_is_end(root->NE) && root->NE->pt != NULL)
        {
            double dist = point2d_distance(pt, root->NE->pt);
            if (dist < *d)
            {
                *d = dist;
                neighbor->x = root->NE->pt->x;
                neighbor->y = root->NE->pt->y;
            }
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
            }
            free(NE_ll);
            free(NE_ur);
        }
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
    return NULL;
}