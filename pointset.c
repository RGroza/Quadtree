#include <stdbool.h>
#include <stddef.h>

#include "point2d.h"
#include "pointset.h"

struct _pointset
{
    double x;
    double y;
} typedef pointset;


pointset *pointset_create(const point2d *pts, size_t n);
size_t pointset_size(const pointset *t);
bool pointset_add(pointset *t, const point2d *pt);
bool pointset_contains(const pointset *t, const point2d *pt);
void pointset_nearest_neighbor(const pointset *t, const point2d *pt, point2d *neighbor, double *d);
point2d *pointset_k_nearest(const pointset *t, const point2d *pt, size_t k);
void pointset_for_each(const pointset* t, void (*f)(const point2d *, void *), void *arg);
void pointset_destroy(pointset *t);


