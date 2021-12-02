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
pointnode *find_path(pointnode *root, const point2d pt);
pointset *pointset_recursive(const point2d *pts, size_t n);


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

    point2d *NW_list = malloc(sizeof(point2d) * n / 2);
    size_t NW_size = 0;
    point2d *SW_list = malloc(sizeof(point2d) * n / 2);
    size_t SW_size = 0;
    for (int i = 0; i < n / 2; i++)
    {
        if (pts_cpy[i].y > root->y)
        {
            NW_list[NW_size] = pts_cpy[i];
            NW_size++;
        }
        else
        {
            SW_list[SW_size] = pts_cpy[i];
            SW_size++;
        }
    }
    pointset *NW_subtree = pointset_create(NW_list, NW_size);
    root->NW = NW_subtree->root;
    pointset *SW_subtree = pointset_create(SW_list, SW_size);
    root->SW = SW_subtree->root;

    point2d *NE_list = malloc(sizeof(point2d) * n / 2);
    size_t NE_size = 0;
    point2d *SE_list = malloc(sizeof(point2d) * n / 2);
    size_t SE_size = 0;
    for (int i = n / 2; i < n; i++)
    {
        if (pts_cpy[i].y > root->y)
        {
            NE_list[NE_size] = pts_cpy[i];
            NE_size++;
        }
        else
        {
            SE_list[SE_size] = pts_cpy[i];
            SE_size++;
        }
    }
    pointset *NE_subtree = pointset_create(NE_list, NE_size);
    root->NE = NE_subtree->root;
    pointset *SE_subtree = pointset_create(SE_list, SE_size);
    root->SE = SE_subtree->root;

    return result;
}


pointset *pointset_recursive(const point2d *pts, size_t n)
{

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
        if (pt->x < curr_node->x)
        {
            if (pt->y > curr_node->y)
            {
                curr_node = curr_node->NW;
            }
            else
            {
                curr_node = curr_node->SW;
            }
        }
        else
        {
            if (pt->y > curr_node->y)
            {
                curr_node = curr_node->NE;
            }
            else
            {
                curr_node = curr_node->SE;
            }
        }
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
        if (pt->x < curr_node->x)
        {
            if (pt->y > curr_node->y)
            {
                curr_node = curr_node->NW;
            }
            else
            {
                curr_node = curr_node->SW;
            }
        }
        else
        {
            if (pt->y > curr_node->y)
            {
                curr_node = curr_node->NE;
            }
            else
            {
                curr_node = curr_node->SE;
            }
        }
    }

    if (curr_node == NULL)
    {
        return NULL;
    }

    return true;
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

pointnode *find_path(pointnode *root, const point2d *pt)
{
    pointnode *curr_node = root;
        while (curr_node != NULL)
        {
            if (pt->x < curr_node->x)
            {
                if (pt->y > curr_node->y)
                {
                    curr_node = curr_node->NW;
                }
                else
                {
                    curr_node = curr_node->SW;
                }
            }
            else
            {
                if (pt->y > curr_node->y)
                {
                    curr_node = curr_node->NE;
                }
                else
                {
                    curr_node = curr_node->SE;
                }
            }
        }
}