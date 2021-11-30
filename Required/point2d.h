#ifndef __POINT2D_H__
#define __POINT2D_H__


typedef struct _point
{
  double x;
  double y;
} point2d;


/**
 * Returns the Euclidean distance between two points.
 *
 * @param p1 a pointer to a point with finite coordinates, non-NULL
 * @param p2 a pointer to a point with finite coordinates, non-NULL
 * @return the distance between those points
 */
double point2d_distance(const point2d *p1, const point2d *p2);


/**
 * Returns the minimum Euclidean distance between the given point and the
 * rectangle defined by the given lower left and upper right corners.
 * The distance is 0 if the point is inside the rectangle.
 *
 * @param pt a pointner to a point with finite coordinates, non-NULL
 * @param lower_left a pointer to a point, non-NULL
 * @param upper_right a pointer to a point whose coordinates
 * greater than or equal to the corresponding coordinates in the point
 * pointed to by upper_left
 * @return the minimum distance from l1 to the rectangle defined by
 * lower_left and upper_right
 */
double point2d_distance_to_rectangle(const point2d *pt, const point2d *lower_left, const point2d *upper_right);


#endif
