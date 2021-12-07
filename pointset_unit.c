#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "point2d.h"
#include "pointset.h"

void test_add(const point2d *pt, size_t n);
void test_size(const point2d *build_pts, size_t n, const point2d *add_pts,
               size_t m);
void test_build(const point2d *pt, size_t n);
void test_contains(size_t n);
void test_for_each(size_t n);
void test_nearest(const point2d *points, size_t n, const point2d *query);
void test_k_nearest(const point2d *points, size_t n, const point2d *query,
                    size_t k);
void test_nearest_after_build(const point2d *points, size_t n,
                              const point2d *query);
void test_nearest_random(size_t n);
void test_k_nearest_random(size_t n, size_t k);
void test_add_unbalanced(size_t n);
void test_add_time_random(size_t n, int on, double x_scale, double y_scale);
void test_build_time_random(size_t n, int on, double x_scale, double y_scale);
void test_nearest_time_random(size_t n, size_t m, int on, double x_scale,
                              double y_scale);
void test_k_nearest_time_random(size_t n, size_t m, size_t k, int on,
                                double x_scale, double y_scale);

point2d *make_random_points(size_t n, double x_scale, double y_scale);
void print_point_formatted(const point2d *pt, void *a);
void print_point(const point2d *pt, void *a);
void brute_force_nearest_neighbor(const point2d *pts, size_t n,
                                  const point2d *pt, point2d *neighbor,
                                  double *d);
point2d *brute_force_k_nearest(const point2d *pts, size_t n, const point2d *pt,
                               size_t k);

point2d nearest_points[] = {{0, 3}, {100, 0}, {1, 1}, {99, -5}, {101, -4}, {-1, -5000}, {-5000, -5}, {98, -3}, {102, -2}, {-1, -5}};
size_t nearest_count = sizeof(nearest_points) / sizeof(*nearest_points);

#define SMALL_TEST_SIZE (1500)
#define LARGE_TEST_SIZE (1000000)

point2d test_points[] = {{24.904359601287595, -164.679680919231197},
                         {22.1889723646466788, -157.4889113459804237},
                         {22.9975176700462036, -156.5980384054300085},
                         {19.352792614743985, -162.739619141829996},
                         {19.71197032632596, -161.0682580028004942},
                         {24.796794566495814, -157.0503041973512346},
                         {22.858083728266023, -158.3873640532406428},
                         {15.356712575011505, -157.8071684212029604},
                         {23.4204901549550186, -162.370437601685534},
                         {23.8584532617037777, -157.5370774004888363},
                         {22.588669770893219, -157.028378147416816},
                         {19.0969835095968481, -160.17803538487762616},
                         {22.2147131873924506, -158.355708807669218},
                         {16.697782049411713, -164.92397630464659},
                         {20.3756508986791074, -162.999269739764995},
                         {17.079064596253004, -162.023667312991769},
                         {18.8630989313068347, -163.993546207602046},
                         {18.0848192792889675, -162.654559728282388},
                         {21.8331905048543264, -161.9598511838378752},
                         {15.070625181945118, -163.9570156703153074}};
size_t test_count = sizeof(test_points) / sizeof(*test_points);

point2d example_1[] = {
    {15.0, 16.0}, // A
    {16.0, 4.0},  // B
    {4.0, 7.0},   // C
    {17.0, 11.0}, // D
    {16.0, 24.0}, // E
    {23.0, 19.0}, // F
    {3.0, 21.0},  // G
    {8.0, 12.0},  // H
    {22.0, 29.0}, // I
    {8.0, 26.0},  // J
    {11.0, 8.0},  // K
};

size_t example_1_count = sizeof(example_1) / sizeof(*example_1);

// point2d example_1[] = {
//     {5.0, 5.0}, // A
//     {1.0, 9.0}, // B
//     {4.0, 9.0}, // C
//     {3.0, 7.0}  // D
// };

// size_t example_1_count = sizeof(example_1) / sizeof(*example_1);

point2d example_2[] = {{6.0, 3.0}, {2.0, 17.0}, {15.0, 20.0}, {2.0, 3.0}, {17.0, 13.0}, {3.0, 9.0}, {10.0, 11.0}, {7.0, 7.0}, {9.0, 12.0}, {16.0, 11.0}, {15.0, 2.0}, {19.0, 4.0}};
size_t example_2_count = sizeof(example_2) / sizeof(*example_2);

point2d example_query_1a = {11.0, 22.0};
point2d example_query_1b = {3.0, 4.0};

int main(int argc, char **argv)
{
  int test = 0;
  /*
  point2d nearest_points_swapped[nearest_count];
  for (size_t i = 0; i < nearest_count; i++)
    {
      nearest_points_swapped[i].x = nearest_points[i].y;
      nearest_points_swapped[i].y = nearest_points[i].x;
    }
  */

  if (argc > 1)
  {
    test = atoi(argv[1]);
  }

  switch (test)
  {
  case 1:
    test_add(test_points, 0);
    break;

  case 2:
    test_add(test_points, 1);
    break;

  case 3:
    test_contains(1);
    break;

  case 4:
    test_add(test_points, test_count / 2);
    break;

  case 44:
    test_size(test_points, 0, test_points, test_count);
    break;

  case 5:
    test_contains(test_count / 2);
    break;

  case 6:
    test_for_each(0);
    break;

  case 7:
    test_for_each(test_count);
    break;

  case 8:
    test_nearest(nearest_points, 0, &nearest_points[0]);
    break;

  case 9:
    // nearest to J is J
    test_nearest(example_1, example_1_count - 1,
                 &example_1[example_1_count - 2]);
    break;

  case 10:
    // excluding K as in first class example
    test_nearest(example_1, example_1_count - 1, &example_query_1a);
    break;

  case 11:
    test_nearest(example_1, example_1_count, &example_query_1b);
    break;

  case 12:
    test_build(test_points, test_count);
    break;

  case 13:
    test_nearest_after_build(test_points, test_count,
                             &test_points[test_count / 2]);
    break;

  case 14:
  {
    size_t n = LARGE_TEST_SIZE;
    if (argc > 2)
    {
      n = atoi(argv[2]);
      if (argc > 3)
      {
        srand(atoi(argv[3]));
      }
    }
    test_nearest_random(n);
  }
  break;

  case 150:
    test_k_nearest(example_1, example_1_count, &example_query_1a, 3);
    break;

  case 15:
  {
    size_t n = LARGE_TEST_SIZE;
    size_t k = 1;
    if (argc > 2)
    {
      n = atoi(argv[2]);
      if (argc > 3)
      {
        k = atoi(argv[3]);

        if (argc > 4)
        {
          srand(atoi(argv[4]));
        }
      }
    }
    test_k_nearest_random(n, k);
  }
  break;

  case 16:
    test_add(example_1, example_1_count);
    break;

  case 17:
    test_add(example_2, example_2_count);
    break;

  case 18:
    test_build(example_1, example_1_count);
    break;

  case 19:
    test_build(example_2, example_2_count);
    break;

  case 20:
    test_nearest_after_build(example_1, example_1_count, &example_query_1a);
    break;

  case 21:
    test_nearest_after_build(example_1, example_1_count, &example_query_1b);
    break;

  case 22:
    // this will be run with a lower limit than normal on the stack size
    test_add_unbalanced(SMALL_TEST_SIZE);
    break;

  case 23:
  {
    int on = true;
    size_t n = LARGE_TEST_SIZE;

    if (argc > 2)
    {
      srand(atoi(argv[2]));

      if (argc > 3)
      {
        n = atoi(argv[3]);
        if (argc > 4)
        {
          on = atoi(argv[4]);
        }
      }
    }
    test_add_time_random(n, on, 100.0, 100.0);
  }
  break;

  case 24:
  {
    int on = true;
    size_t n = LARGE_TEST_SIZE;

    if (argc > 2)
    {
      srand(atoi(argv[2]));

      if (argc > 3)
      {
        n = atoi(argv[3]);
        if (argc > 4)
        {
          on = atoi(argv[4]);
        }
      }
    }
    test_build_time_random(n, on, 100.0, 100.0);
  }
  break;

  case 25:
  {
    int on = true;
    size_t n = LARGE_TEST_SIZE;
    size_t m = 100;

    if (argc > 2)
    {
      srand(atoi(argv[2]));

      if (argc > 3)
      {
        m = atoi(argv[3]);
        if (argc > 4)
        {
          n = atoi(argv[4]);
          if (argc > 5)
          {
            on = atoi(argv[5]);
          }
        }
      }
    }
    test_nearest_time_random(n, m, on, 100.0, 100.0);
  }
  break;

  case 26:
  {
    int on = true;
    size_t n = LARGE_TEST_SIZE;
    size_t m = 100;
    size_t k = 5;

    if (argc > 2)
    {
      srand(atoi(argv[2]));

      if (argc > 3)
      {
        m = atoi(argv[3]);
        if (argc > 4)
        {
          k = atoi(argv[4]);
          if (argc > 5)
          {
            n = atoi(argv[5]);
            if (argc > 6)
            {
              on = atoi(argv[6]);
            }
          }
        }
      }
    }
    test_k_nearest_time_random(n, m, k, on, 100.0, 100.0);
  }
  break;

  default:
    fprintf(stderr, "USAGE: %s test-number\n", argv[0]);
    return 1;
  }

  return 0;
}

void print_point(const point2d *pt, void *a)
{
  printf("%f %f\n", pt->x, pt->y);
}

void test_add(const point2d *test_points, size_t n)
{
  pointset *t = pointset_create(NULL, 0);

  if (t == NULL)
  {
    printf("FAILED -- could not create tree\n");
    return;
  }

  for (size_t i = 0; i < n; i++)
  {
    if (!pointset_add(t, &test_points[i]))
    {
      printf("FAILED -- could not add point %f %f\n", test_points[i].x,
             test_points[i].y);
      pointset_destroy(t);
      return;
    }

    if (pointset_add(t, &test_points[i]))
    {
      printf("FAILED -- added duplicate point %f %f\n", test_points[i].x,
             test_points[i].y);
      pointset_destroy(t);
      return;
    }
  }

  pointset_destroy(t);
  printf("PASSED\n");
}

void test_build(const point2d *points, size_t n)
{
  pointset *t = pointset_create(points, n);

  if (t == NULL)
  {
    printf("FAILED -- could not create tree\n");
    return;
  }

  for (size_t i = 0; i < n; i++)
  {
    if (!pointset_contains(t, &points[i]))
    {
      printf("FAILED -- missing point %f %f\n", points[i].x, points[i].y);
      pointset_destroy(t);
      return;
    }
  }

  pointset_destroy(t);
  printf("PASSED\n");
}

void test_size(const point2d *build_pts, size_t n, const point2d *add_pts,
               size_t m)
{
  pointset *t = pointset_create(build_pts, n);

  if (t == NULL)
  {
    printf("FAILED -- could not create tree\n");
    return;
  }

  if (pointset_size(t) != n)
  {
    printf("FAILED -- size after building with %zu points is %zu\n", n,
           pointset_size(t));
    pointset_destroy(t);
    return;
  }

  for (size_t i = 0; i < m; i++)
  {
    if (!pointset_add(t, &add_pts[i]))
    {
      printf("FAILED -- could not add point %lf %lf\n", add_pts[i].x,
             add_pts[i].y);
      pointset_destroy(t);
      return;
    }
  }

  if (pointset_size(t) != n + m)
  {
    printf(
        "FAILED -- size after building with %zu points and adding %zu is %zu\n",
        n, m, pointset_size(t));
    pointset_destroy(t);
    return;
  }

  pointset_destroy(t);
  printf("PASSED\n");
}

void test_contains(size_t n)
{
  pointset *t = pointset_create(NULL, 0);

  if (t == NULL)
  {
    printf("FAILED -- could not create tree\n");
    return;
  }

  for (size_t i = 0; i < n; i++)
  {
    if (!pointset_add(t, &test_points[i]))
    {
      printf("FAILED -- could not add point %f %f\n", test_points[i].x,
             test_points[i].y);
      pointset_destroy(t);
      return;
    }
  }

  for (size_t i = 0; i < n; i++)
  {
    if (!pointset_contains(t, &test_points[i]))
    {
      printf("FAILED -- missing point %f %f\n", test_points[i].x,
             test_points[i].y);
      pointset_destroy(t);
      return;
    }
  }

  for (size_t i = n; i < test_count; i++)
  {
    if (pointset_contains(t, &test_points[i]))
    {
      printf("FAILED -- contains point %f %f\n", test_points[i].x,
             test_points[i].y);
      pointset_destroy(t);
      return;
    }
  }

  pointset_destroy(t);
  printf("PASSED\n");
}

void test_nearest(const point2d *points, size_t n, const point2d *query)
{
  // build the tree by adding points one-by-one
  pointset *t = pointset_create(NULL, 0);
  for (size_t i = 0; i < n; i++)
  {
    pointset_add(t, &points[i]);
  }

  // get the nearest neighbor to our query point
  point2d nearest;
  double distance;
  pointset_nearest_neighbor(t, query, &nearest, &distance);

  // output the result
  printf("Distance: %f\n", distance);
  if (distance < INFINITY)
  {
    print_point_formatted(&nearest, "Point: %f %f\n");
  }

  pointset_destroy(t);
}

void test_k_nearest(const point2d *points, size_t n, const point2d *query,
                    size_t k)
{
  // build the tree by adding points one-by-one
  pointset *t = pointset_create(NULL, 0);
  for (size_t i = 0; i < n; i++)
  {
    pointset_add(t, &points[i]);
  }

  // get the k nearest neighbors to our query point
  point2d *nearest = pointset_k_nearest(t, query, k);

  // output the results
  for (size_t i = 0; i < k && i < n; i++)
  {
    print_point_formatted(nearest + i, "Point: %f %f\n");
  }

  free(nearest);
  pointset_destroy(t);
}

void test_nearest_after_build(const point2d *points, size_t n,
                              const point2d *query)
{
  // build the tree all at once (should be balanced)
  pointset *t = pointset_create(points, n);

  // get the nearest neighbor to our query point
  point2d nearest;
  double distance;
  pointset_nearest_neighbor(t, query, &nearest, &distance);

  // output the result
  printf("Distance: %f\n", distance);
  if (distance < INFINITY)
  {
    print_point_formatted(&nearest, "Point: %f %f\n");
  }

  pointset_destroy(t);
}

void test_nearest_random(size_t n)
{
  // make an array of random points
  point2d *random_points = malloc(sizeof(*random_points) * (n + 1));
  if (random_points == NULL)
  {
    printf("FAILED -- could not allocate test points\n");
    return;
  }

  for (size_t i = 0; i < n + 1; i++)
  {
    random_points[i].x = (double)rand() / INT_MAX * 1000.0;
    random_points[i].y = (double)rand() / INT_MAX * 1000.0;
  }
  point2d query = random_points[n];

  // build a tree using those points
  pointset *random = pointset_create(random_points, n);

  // get the nearest neighbor using the pointset algorithm
  point2d neighbor;
  double distance;
  pointset_nearest_neighbor(random, &query, &neighbor, &distance);

  // get the nearest neighbor by brute force
  point2d bf_neighbor;
  double bf_distance;
  brute_force_nearest_neighbor(random_points, n, &query, &bf_neighbor,
                               &bf_distance);

  // free points and tree
  free(random_points);
  pointset_destroy(random);

  // check the results
  if (distance != bf_distance ||
      (distance < INFINITY &&
       (neighbor.x != bf_neighbor.x || neighbor.y != bf_neighbor.y)))
  {
    printf("FAILED -- for %f %f got %f to %f %f instead of %f to %f %f\n",
           query.x, query.y, distance, neighbor.x, neighbor.y, bf_distance,
           bf_neighbor.x, bf_neighbor.y);
  }
  else
  {
    printf("PASSED\n");
  }
}

void test_k_nearest_random(size_t n, size_t k)
{
  // make an array of random points
  point2d *random_points = malloc(sizeof(*random_points) * (n + 1));
  if (random_points == NULL)
  {
    printf("FAILED -- could not allocate test points\n");
    return;
  }

  for (size_t i = 0; i < n + 1; i++)
  {
    random_points[i].x = (double)rand() / INT_MAX * 1000.0;
    random_points[i].y = (double)rand() / INT_MAX * 1000.0;
  }
  point2d query = random_points[n];

  // build a tree using those points
  pointset *random = pointset_create(random_points, n);

  // get the nearest neighbor using the pointset algorithm
  point2d *nearest = pointset_k_nearest(random, &query, k);

  // check that returned array is non-NULL
  if (nearest == NULL)
  {
    printf("FAILED -- returned array is NULL\n");
    free(random_points);
    pointset_destroy(random);
    return;
  }

  // get the nearest neighbors by brute force
  point2d *nearest_bf = brute_force_k_nearest(random_points, n, &query, k);

  // check that returned array is non-NULL
  if (nearest == NULL)
  {
    printf("FAILED -- returned array is NULL\n");
    free(nearest);
    free(random_points);
    pointset_destroy(random);
    return;
  }

  // free points and tree
  free(random_points);
  pointset_destroy(random);

  // check the results
  size_t i = 0;
  while (i < n && i < k && point2d_distance(nearest + i, nearest_bf + i) == 0)
  {
    i++;
  }

  if (i < n && i < k)
  {
    printf(
        "FAILED -- for nearest to %lf %lf at index %zuth got %f %f (d=%lf) "
        "instead of %f %f (d=%lf)\n",
        query.x, query.y, i, nearest[i].x, nearest[i].y,
        point2d_distance(&query, nearest + i), nearest_bf[i].x, nearest_bf[i].y,
        point2d_distance(&query, nearest_bf + i));
    free(nearest);
    free(nearest_bf);
  }
  else
  {
    free(nearest);
    free(nearest_bf);

    printf("PASSED\n");
  }
}

void test_for_each(size_t n)
{
  pointset *t = pointset_create(NULL, 0);

  if (t == NULL)
  {
    printf("FAILED -- could not create tree\n");
    return;
  }
  else
  {
    for (size_t i = 0; i < n; i++)
    {
      if (!pointset_add(t, &test_points[i]))
      {
        printf("FAILED -- could not add point %f %f\n", test_points[i].x,
               test_points[i].y);
        pointset_destroy(t);
        return;
      }
    }
  }

  pointset_for_each(t, print_point_formatted, "%.3f %.3f\n");

  pointset_destroy(t);
}

void print_point_formatted(const point2d *pt, void *a)
{
  const char *format = a;

  printf(format, pt->x, pt->y);
}

void brute_force_nearest_neighbor(const point2d *pts, size_t n,
                                  const point2d *pt, point2d *neighbor,
                                  double *d)
{
  if (n == 0)
  {
    *d = INFINITY;
  }
  else
  {
    *d = point2d_distance(&pts[0], pt);
    *neighbor = pts[0];
    for (size_t i = 1; i < n; i++)
    {
      double curr_d = point2d_distance(&pts[i], pt);
      if (curr_d < *d)
      {
        *d = curr_d;
        *neighbor = pts[i];
      }
    }
  }
}

point2d *brute_force_k_nearest(const point2d *pts, size_t n, const point2d *pt,
                               size_t k)
{
  // makin' copies (so we can shuffle the points around without disturbing pts)
  point2d *copy = malloc(sizeof(*copy) * n);
  if (copy == NULL)
  {
    return NULL;
  }
  memcpy(copy, pts, sizeof(*copy) * n);

  // first k rounds of selection sort to get the k closest
  for (size_t i = 0; i < k && i < n; i++)
  {
    double min = point2d_distance(pt, copy + i);
    size_t min_index = i;
    for (size_t j = i + 1; j < n; j++)
    {
      double dist;
      if ((dist = point2d_distance(pt, copy + j)) < min)
      {
        min = dist;
        min_index = j;
      }
    }
    point2d temp = copy[min_index];
    copy[min_index] = copy[i];
    copy[i] = temp;
  }

  // shrink (or grow, if n < k) the array to the first k points
  point2d *shorter = realloc(copy, sizeof(*copy) * k);
  if (shorter == NULL)
  {
    free(copy);
    return NULL;
  }
  return shorter;
}

void test_add_unbalanced(size_t n)
{
  pointset *t = pointset_create(NULL, 0);

  if (t == NULL)
  {
    printf("FAILED -- could not create tree\n");
    return;
  }
  else
  {
    for (size_t i = 0; i < n; i++)
    {
      point2d pt = {i, i};
      if (!pointset_add(t, &pt))
      {
        printf("FAILED -- could not add point %f %f\n", pt.x, pt.y);
        pointset_destroy(t);
        return;
      }
    }
  }

  pointset_destroy(t);
  printf("PASSED\n");
}

void test_add_time_random(size_t n, int on, double x_scale, double y_scale)
{
  point2d *random_points = make_random_points(n, x_scale, y_scale);
  if (random_points == NULL)
  {
    printf("FAILED -- could not create points\n");
    return;
  }

  if (on)
  {
    pointset *t = pointset_create(NULL, 0);

    if (t == NULL)
    {
      printf("FAILED -- could not build tree\n");
      free(random_points);
      return;
    }

    for (size_t i = 0; i < n; i++)
    {
      pointset_add(t, &random_points[i]);
    }

    for (size_t i = 0; i < n; i++)
    {
      if (!pointset_contains(t, &random_points[i]))
      {
        printf("FAILED -- lost point (%f, %f)\n", random_points[i].x,
               random_points[i].y);
        pointset_destroy(t);
        free(random_points);
        return;
      }
    }

    pointset_destroy(t);
  }

  free(random_points);
}

void test_build_time_random(size_t n, int on, double x_scale, double y_scale)
{
  point2d *random_points = make_random_points(n, x_scale, y_scale);
  if (random_points == NULL)
  {
    printf("FAILED -- could not create points\n");
    return;
  }

  if (on)
  {
    pointset *t = pointset_create(random_points, n);

    if (t == NULL)
    {
      printf("FAILED -- could not build tree\n");
      free(random_points);
      return;
    }

    for (size_t i = 0; i < n; i++)
    {
      if (!pointset_contains(t, &random_points[i]))
      {
        printf("FAILED -- lost point (%f, %f)\n", random_points[i].x,
               random_points[i].y);
        pointset_destroy(t);
        free(random_points);
        return;
      }
    }

    pointset_destroy(t);
  }

  free(random_points);
}

void test_nearest_time_random(size_t n, size_t m, int on, double x_scale,
                              double y_scale)
{
  point2d *random_points = make_random_points(n + m, x_scale, y_scale);
  if (random_points == NULL)
  {
    printf("FAILED -- could not create points\n");
    return;
  }

  pointset *t = pointset_create(random_points, n);

  if (t == NULL)
  {
    printf("FAILED -- could not build tree\n");
    free(random_points);
    return;
  }

  if (on)
  {
    for (size_t i = 0; i < m; i++)
    {
      point2d nearest;
      double dist;
      pointset_nearest_neighbor(t, &random_points[n + i], &nearest, &dist);
    }
  }

  pointset_destroy(t);
  free(random_points);
}

void test_k_nearest_time_random(size_t n, size_t m, size_t k, int on,
                                double x_scale, double y_scale)
{
  point2d *random_points = make_random_points(n + m, x_scale, y_scale);
  if (random_points == NULL)
  {
    printf("FAILED -- could not create points\n");
    return;
  }

  pointset *t = pointset_create(random_points, n);

  if (t == NULL)
  {
    printf("FAILED -- could not build tree\n");
    free(random_points);
    return;
  }

  if (on)
  {
    for (size_t i = 0; i < m; i++)
    {
      point2d *nearest = pointset_k_nearest(t, &random_points[n + i], k);
      if (nearest == NULL)
      {
        printf("FAILED -- could not find k nearest neighbors\n");
        pointset_destroy(t);
        free(random_points);
        return;
      }
      free(nearest);
    }
  }

  pointset_destroy(t);
  free(random_points);
}

point2d *make_random_points(size_t n, double x_scale, double y_scale)
{
  point2d *random_points = malloc(sizeof(*random_points) * n);
  if (random_points != NULL)
  {
    for (size_t i = 0; i < n; i++)
    {
      random_points[i].x = (double)rand() / RAND_MAX * x_scale + 1.0 / (i + 1);
      random_points[i].y = (double)rand() / RAND_MAX * y_scale + 1.0 / (i + 1);
    }
  }
  return random_points;
}
