#include <stdbool.h>
#include <stddef.h>

#include "point2d.h"
#include "pointset.h"

struct _pointset
{
    pointnode *root;
    size_t size;
} typedef pointset;

struct _pointnode
{
    double x;
    double y;
    pointnode *NW;
    pointnode *SW;
    pointnode *SE;
    pointnode *NE;
} typedef pointnode;

struct _point2dNode
{
    point2d pt;
    point2dNode *next;
} typedef point2dNode;


pointset *pointset_create(const point2d *pts, size_t n);
size_t pointset_size(const pointset *t);
bool pointset_add(pointset *t, const point2d *pt);
bool pointset_contains(const pointset *t, const point2d *pt);
void pointset_nearest_neighbor(const pointset *t, const point2d *pt, point2d *neighbor, double *d);
point2d *pointset_k_nearest(const pointset *t, const point2d *pt, size_t k);
void pointset_for_each(const pointset* t, void (*f)(const point2d *, void *), void *arg);
void pointset_destroy(pointset *t);

int compare_points(const point2d pt1, const point2d pt2);
pointnode recursive_create(const point2d *pts, size_t n);


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

    qsort(pts, n, sizeof(point2d), compare_points);

    pointnode *root = malloc(sizeof(pointnode));
    root->x = pts[n / 2].x;
    root->y = pts[n / 2].y;
    result->root = root;

    // root->NW = recursive_create()

    point2dNode *NW_list = malloc(sizeof(point2dNode));
    point2dNode *SW_list = malloc(sizeof(point2dNode));
    for (int i = 0; i < n / 2; i++)
    {
        point2dNode *NW_curr = malloc(sizeof(point2dNode));
        point2dNode *SW_curr = malloc(sizeof(point2dNode));

        if (pts[i].y > root->y)
        {
            NW_list->pt = pts[i];
            NW_curr = NW_curr->next;
        }
        else
        {
            SW_list->pt = pts[i];
            SW_curr = SW_curr->next;
        }
    }
}


pointnode recursive_create(const point2d *pts, size_t n)
{

}


int comapre_points(const point2d pt1, const point2d pt2)
{
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