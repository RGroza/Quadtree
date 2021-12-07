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
pointnode *pointset_find_node(const pointset *t, const point2d *pt);
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
bool node_is_leaf(pointnode *node);

pointnode *create_new_node(const point2d *pt);
bool is_NW(pointnode *curr, const point2d *pt);
bool is_SW(pointnode *curr, const point2d *pt);
bool is_SE(pointnode *curr, const point2d *pt);
bool is_NE(pointnode *curr, const point2d *pt);
void set_NW_region(pointnode *parent, pointnode *child);
void set_SW_region(pointnode *parent, pointnode *child);
void set_SE_region(pointnode *parent, pointnode *child);
void set_NE_region(pointnode *parent, pointnode *child);

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

    // pointset_add(result, &pts_cpy[n / 2]);

    // for (int i = 0; i < n / 2; i++)
    // {
    //     pointset_add(result, &pts_cpy[i]);
    // }
    // for (int i = n / 2 + 1; i < n; i++)
    // {
    //     pointset_add(result, &pts_cpy[i]);
    // }

    result->root = create_new_node(&pts_cpy[n / 2]);
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
        if (pts[i].y >= root->pt->y)
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
        pointnode *NW_child = create_new_node(&NW_list[NW_size / 2]);
        set_NW_region(root, NW_child);

        root->NW = NW_child;

        if (NW_size > 1)
        {
            pointset_recursive(NW_child, NW_list, NW_size);
        }
    }
    free(NW_list);

    if (SW_size > 0)
    {
        pointnode *SW_child = create_new_node(&SW_list[SW_size / 2]);
        set_SW_region(root, SW_child);

        root->SW = SW_child;

        if (SW_size > 1)
        {
            pointset_recursive(SW_child, SW_list, SW_size);
        }
    }
    free(SW_list);

    point2d *SE_list = malloc(sizeof(point2d) * n / 2);
    size_t SE_size = 0;
    point2d *NE_list = malloc(sizeof(point2d) * n / 2);
    size_t NE_size = 0;
    for (int i = n / 2 + 1; i < n; i++)
    {
        if (pts[i].y < root->pt->y)
        {
            SE_list[SE_size] = pts[i];
            SE_size++;
        }
        else
        {
            NE_list[NE_size] = pts[i];
            NE_size++;
        }
    }

    if (SE_size > 0)
    {
        pointnode *SE_child = create_new_node(&SE_list[SE_size / 2]);
        set_SE_region(root, SE_child);

        root->SE = SE_child;

        if (SE_size > 1)
        {
            pointset_recursive(SE_child, SE_list, SE_size);
        }
    }
    free(SE_list);

    if (NE_size > 0)
    {
        pointnode *NE_child = create_new_node(&NE_list[NW_size / 2]);
        set_NE_region(root, NE_child);

        root->NE = NE_child;

        if (NE_size > 1)
        {
            pointset_recursive(NE_child, NE_list, NE_size);
        }
    }
    free(NE_list);
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

    pointnode *parent = pointset_find_node(t, pt);
    if (parent->pt->x == pt->x && parent->pt->y == pt->y)
    {
        return false;
    }

    pointnode *child = create_new_node(pt);
    if (child == NULL)
    {
        return false;
    }

    if (is_NW(parent, pt))
    {
        parent->NW = child;
        set_NW_region(parent, child);
    }
    else if (is_SW(parent, pt))
    {
        parent->SW = child;
        set_SW_region(parent, child);
    }
    else if (is_SE(parent, pt))
    {
        parent->SE = child;
        set_SE_region(parent, child);
    }
    else
    {
        parent->NE = child;
        set_NE_region(parent, child);
    }

    t->size++;

    return true;
}

pointnode *create_new_node(const point2d *pt)
{
    pointnode *new = malloc(sizeof(pointnode));
    new->pt = malloc(sizeof(point2d));
    new->pt->x = pt->x;
    new->pt->y = pt->y;
    new->NW = NULL;
    new->SW = NULL;
    new->SE = NULL;
    new->NE = NULL;
    new->region_ll = NULL;
    new->region_ur = NULL;

    return new;
}

bool is_NW(pointnode *curr, const point2d *pt)
{
    return pt->x <= curr->pt->x && pt->y >= curr->pt->y;
}

bool is_SW(pointnode *curr, const point2d *pt)
{
    return pt->x <= curr->pt->x && pt->y < curr->pt->y;
}

bool is_SE(pointnode *curr, const point2d *pt)
{
    return pt->x > curr->pt->x && pt->y < curr->pt->y;
}

bool is_NE(pointnode *curr, const point2d *pt)
{
    return pt->x > curr->pt->x && pt->y >= curr->pt->y;
}

void set_NW_region(pointnode *parent, pointnode *child)
{
    pointset_set_region(child, (point2d){parent->region_ll->x, parent->pt->y},
                        (point2d){parent->pt->x, parent->region_ur->y});
}

void set_SW_region(pointnode *parent, pointnode *child)
{
    pointset_set_region(child, *parent->region_ll, *parent->pt);
}

void set_SE_region(pointnode *parent, pointnode *child)
{
    pointset_set_region(child, (point2d){parent->pt->x, parent->region_ll->y}, (point2d){parent->region_ur->x, parent->pt->y});
}

void set_NE_region(pointnode *parent, pointnode *child)
{
    pointset_set_region(child, *parent->pt, *parent->region_ur);
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

pointnode *pointset_find_node(const pointset *t, const point2d *pt)
{
    if (t->root->pt == NULL)
    {
        return NULL;
    }

    pointnode *curr_node = t->root;
    pointnode *prev_node = curr_node;
    while (curr_node != NULL && (curr_node->pt->x != pt->x || curr_node->pt->y != pt->y))
    {
        prev_node = curr_node;
        curr_node = find_path(curr_node, pt);
    }

    // Found node that contains same point as pt
    if (curr_node != NULL)
    {
        return curr_node;
    }
    // Otherwise, return the leaf node
    return prev_node;
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
    if (is_NW(root, pt))
    {
        return root->NW;
    }
    if (is_SW(root, pt))
    {
        return root->SW;
    }
    if (is_SE(root, pt))
    {
        return root->SE;
    }
    if (is_NE(root, pt))
    {
        return root->NE;
    }
}

void pointset_destroy(pointset *t)
{
    pointnode_delete(t->root);
    free(t);
}

void pointnode_delete(pointnode *root)
{
    pointnode *children[4] = {root->NW, root->SW, root->SE, root->NE};

    for (int i = 0; i < 4; i++)
    {
        if (children[i] != NULL)
        {
            pointnode_delete(children[i]);
        }
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
    pointnode *children[4] = {root->NW, root->SW, root->SE, root->NE};

    for (int i = 0; i < 4; i++)
    {
        if (children[i] != NULL)
        {
            recursive_search(children[i], f, arg);
        }
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
        return;
    }

    if (pointset_contains(t, pt))
    {
        *d = 0;
        neighbor->x = pt->x;
        neighbor->y = pt->y;
        return;
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
    pointnode *children[4] = {root->NW, root->SW, root->SE, root->NE};

    for (int i = 0; i < 4; i++)
    {
        if (children[i] != NULL)
        {
            if (node_is_leaf(children[i]) && children[i]->pt != NULL)
            {
                check_distance(children[i], pt, neighbor, d);
            }
            else
            {
                if (point2d_distance_to_rectangle(pt, children[i]->region_ll, children[i]->region_ur) < *d)
                {
                    node_nearest_neighbor(children[i], pt, neighbor, d);
                    check_distance(root, pt, neighbor, d);
                }
            }
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

bool node_is_leaf(pointnode *node)
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
    point2d *nearest = malloc(sizeof(point2d) * k);
    size_t found = 0;

    pqueue_enqueue(q, 0, t->root, 1);
    while (pqueue_size(q) != 0 && found < k)
    {
        size_t *ID = malloc(sizeof(size_t));
        void *item = pqueue_dequeue(q, NULL, ID);
        if (*ID == 0)
        {
            point2d new_pt = *(point2d *)item;
            nearest[found] = new_pt;
            found++;
        }
        else
        {
            pointnode *node = (pointnode *)item;
            double dist = point2d_distance(pt, node->pt);
            pqueue_enqueue(q, dist, node->pt, 0);

            pointnode *children[4] = {node->NW, node->SW, node->SE, node->NE};
            for (int i = 0; i < 4; i++)
            {
                if (children[i] != NULL)
                {
                    pqueue_enqueue(q, point2d_distance_to_rectangle(pt, children[i]->region_ll, children[i]->region_ur), children[i], 1);
                }
            }
        }
        free(ID);
    }
    pqueue_destroy(q, NULL);

    return nearest;
}