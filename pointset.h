#ifndef __POINTSET_H__
#define __POINTSET_H__

#include <stdbool.h>
#include <stddef.h>

#include "point2d.h"


/**
 * A set of Cartesian coordinates implemented using a quadtree
 */
typedef struct _pointset pointset;


/**
 * Creates a point set containing copies of the points in the
 * given array of points.  If the array is NULL and n is 0 then
 * the returned tree is empty.  If the tree could not be created
 * then the returned value is NULL.  If the return value is not
 * NULL then it is the caller's responsibility to eventually
 * destroy the point set.
 *
 * @param pts an array of unique valid points, non-NULL if n != 0
 * @param n the number of points in that array, or 0 if pts is NULL
 * @return a pointer to the newly created collection of points, or NULL
 * if it could not be created
 */
pointset *pointset_create(const point2d *pts, size_t n);


/**
 * Returns the number of points in the given set.
 *
 * @param a pointer to a valid pointset, non-NULL
 * @return the number of points in that set
 */
size_t pointset_size(const pointset *t);


/**
 * Adds a copy of the given point to the given pointset.  There is no
 * effect if the point is already in the tree.  The tree need not be
 * balanced after the add.  The return value is true if the point was
 * added successfully and false otherwise (if the point was already in the
 * tree or if there was a memory allocation error when trying to add).
 *
 * @param t a pointer to a valid pointset, non-NULL
 * @param pt a pointer to a valid point, non-NULL
 * @return true if and only if the point was successfully added
 */
bool pointset_add(pointset *t, const point2d *pt);


/**
 * Determines if the given tree contains a point with the same coordinates
 * as the given point.
 *
 * @param t a pointer to a valid pointset, non-NULL
 * @param pt a pointer to a valid point, non-NULL
 * @return true if and only of the tree contains the location
 */
bool pointset_contains(const pointset *t, const point2d *pt);


/**
 * Copies the nearest neighbor to the given point among points in the
 * given point set to the given point.  Ties are broken arbitrarily.
 * There is no change to neighbor and distance is set to infinity if
 * the set is empty.  If p is equal to a point in the set then p is
 * copied into neighbor and distance is set to zero.
 *
 * @param t a pointer to a valid pointset, non-NULL
 * @param pt a pointer to a valid point with finite coordinates, non-NULL
 * @param d a pointer to a double, non-NULL
 * @param neighbor a pointer to a valid point, non-NULL
 */
void pointset_nearest_neighbor(const pointset *t, const point2d *pt, point2d *neighbor, double *d);


/**
 * Returns the k nearest neighbors to the given point among points in the
 * given point set.  Ties are broken arbitrarily.  The k nearest neighbors
 * are returned in array in increasing order of distance.  If the set contains
 * fewer than k points then only the elements in the returned array up
 * to the size of the set will be valid.  If there is an allocation error
 * during the operation, then the return value is NULL.  It is the caller's
 * responsibility to free the returned array.
 *
 * @param t a pointer to a valid pointset, non-NULL
 * @param pt a pointer to a valid point with finite coordinates, non-NULL
 * @param k a positive integer
 */
point2d *pointset_k_nearest(const pointset *t, const point2d *pt, size_t k);


/**
 * Passes the points in the given tree to the given function
 * in an arbitrary order.  The last argument to this function is also passed
 * to the given function along with each point.
 *
 * @param t a pointer to a valid pointset, non-NULL
 * @param f a pointer to a function that takes a location and an extra
 * argument, and does not modify t, non-NULL
 * @param arg a pointer
 */
void pointset_for_each(const pointset* t, void (*f)(const point2d *, void *), void *arg);


/**
 * Destroys the given pointset.  The tree is invalid after being destroyed.
 *
 * @param t a pointer to a valid pointset, non-NULL
 */
void pointset_destroy(pointset *t);


#endif
