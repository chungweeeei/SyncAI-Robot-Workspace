#ifndef SYNCAI_AMCL__PORTABLE_UTILS_HPP_
#define SYNCAI_AMCL__PORTABLE_UTILS_HPP_

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef HAVE_DRAND48
// Some system (e.g., Windows) doesn't come with drand48(), srand48().
// Use rand, and srand for such system.
static double drand48(void)
{
  return ((double)rand()) / RAND_MAX;  // NOLINT
}

static void srand48(long int seedval)  // NOLINT
{
  srand(seedval);
}
#endif

#ifdef __cplusplus
}
#endif

#endif  // SYNCAI_AMCL__PORTABLE_UTILS_HPP_
