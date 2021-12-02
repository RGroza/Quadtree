#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "point2d.h"
#include "pointset.h"

typedef struct _pointset
{
    struct _pointnode *root;
    size_t size;
} pointset;

typedef struct _pointnode
{
    double x;
    double y;
    struct _pointnode *NW;
    struct _pointnode *SW;
    struct _pointnode *SE;
    struct _pointnode *NE;
} pointnode;


pointset *pointset_create(const point2d *pts, size_t n);
size_t pointset_size(const pointset *t);
bool pointset_add(pointset *t, const point2d *pt);
bool pointset_contains(const pointset *t, const point2d *pt);
void pointset_nearest_neighbor(const pointset *t, const point2d *pt, point2d *neighbor, double *d);
point2d *pointset_k_nearest(const pointset *t, const point2d *pt, size_t k);
void pointset_for_each(const pointset* t, void (*f)(const point2d *, void *), void *arg);
void pointset_destroy(pointset *t);

int compare_points(const void *p1, const void *p2);
pointnode *find_path(pointnode *root, const point2d *pt);
void pointset_recursive(pointnode *root, const point2d *pts, size_t n);


pointset *pointset_create(const point2d *pts, size_t n)
{
    pointset *result = malloc(sizeof(pointset));
    if (result == NULL)
    {
        return NULL;
    }

    result->size = n;
    if (pts == NULL && n == 0)
    {
        result->root = NULL;
        return result;
    }

    point2d *pts_cpy = malloc(sizeof(point2d) * n);
    memcpy(pts_cpy, pts, sizeof(point2d) * n);

    qsort(pts_cpy, n, sizeof(point2d), compare_points);

    pointnode *root = malloc(sizeof(pointnode));
    root->x = pts_cpy[n / 2].x;
    root->y = pts_cpy[n / 2].y;
    result->root = root;

    pointset_recursive(root, pts_cpy, n);

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
        if (pts[i].y > root->y)
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
        NW_child->x = NW_list[n / 2].x;
        NW_child->y = NW_list[n / 2].y;

        root->NW = NW_child;
  
        pointset_recursive(NW_child, NW_list, NW_size);
    }
    free(NW_list);

    if (SW_size > 0)
    {
        pointnode *SW_child = malloc(sizeof(pointnode));
        SW_child->x = SW_list[n / 2].x;
        SW_child->y = SW_list[n / 2].y;

        root->SW = SW_child;
  
        pointset_recursive(SW_child, SW_list, SW_size);
    }
    free(SW_list);


    point2d *NE_list = malloc(sizeof(point2d) * n / 2);
    size_t NE_size = 0;
    point2d *SE_list = malloc(sizeof(point2d) * n / 2);
    size_t SE_size = 0;
    for (int i = 0; i < n / 2; i++)
    {
        if (pts[i].y > root->y)
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
        NE_child->x = NE_list[n / 2].x;
        NE_child->y = NE_list[n / 2].y;

        root->NE = NE_child;
  
        pointset_recursive(NE_child, NE_list, NE_size);
    }
    free(NE_list);

    if (SE_size > 0)
    {
        pointnode *SE_child = malloc(sizeof(pointnode));
        SE_child->x = SE_list[n / 2].x;
        SE_child->y = SE_list[n / 2].y;

        root->SE = SE_child;
  
        pointset_recursive(SE_child, SE_list, SE_size);
    }
    free(SE_list);
}


size_t pointset_size(const pointset *t)
{
    return t->size;
}


bool pointset_add(pointset *t, const point2d *pt)
{
    if (pointset_contains(t, pt))
    {
        return false;
    }

    pointnode *curr_node = t->root;
    while (curr_node != NULL)
    {
        find_path(curr_node, pt);
    }

    pointnode *new_node = malloc(sizeof(pointnode));
    if (new_node == NULL)
    {
        return false;
    }

    new_node->x = pt->x;
    new_node->y = pt->y;

    return true;
}


bool pointset_contains(const pointset *t, const point2d *pt)
{
    pointnode *curr_node = t->root;
    while ((curr_node->x != pt->x && curr_node->y != pt->y) || curr_node != NULL)
    {
        curr_node = find_path(curr_node, pt);
    }

    if (curr_node == NULL)
    {
        return NULL;
    }

    return true;
}


int comapre_points(const void *p1, const void *p2)
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
    if (pt->x < root->x)
    {
        if (pt->y > root->y)
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
        if (pt->y > root->y)
        {
            return root->NE;
        }
        else
        {
            return root->SE;
        }
    }
}