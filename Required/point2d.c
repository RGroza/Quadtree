#include <stddef.h>
#include <math.h>

#include "point2d.h"


double point2d_distance(const point2d *p1, const point2d *p2)
{
  return sqrt(pow(p2->x - p1->x, 2) + pow(p2->y - p1->y, 2));
}


double point2d_distance_to_rectangle(const point2d *pt, const point2d *lower_left, const point2d *upper_right)
{
  if (pt->x >= lower_left->x && pt->x <= upper_right->x
      && pt->y >= lower_left->y && pt->y <= upper_right->y)
    {
      // pt is inside rectangle
      return 0;
    }
  
  point2d closest; // the closest point on the rectangle

  if (pt->x < lower_left->x)
    {
      // to the left
      closest.x = lower_left->x;
    }
  else if (pt->x <= upper_right->x)
    {
      // between left and right
      closest.x = pt->x;
    }
  else
    {
      // to the right
      closest.x = upper_right->x;
    }

  if (pt->y < lower_left->y)
    {
      // below
      closest.y = lower_left->y;
    }
  else if (pt->y <= upper_right->y)
    {
      // between top and bottom
      closest.y = pt->y;
    }
  else
    {
      // above
      closest.y = upper_right->y;
    }

  return point2d_distance(pt, &closest);
}
