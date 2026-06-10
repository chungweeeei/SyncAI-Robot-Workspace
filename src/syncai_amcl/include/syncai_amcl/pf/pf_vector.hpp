#ifndef SYNCAI_AMCL__PF__PF_VECTOR_HPP_
#define SYNCAI_AMCL__PF__PF_VECTOR_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

// The basic vector
typedef struct
{
  double v[3];
} pf_vector_t;

// The basic matrix
typedef struct
{
  double m[3][3];
} pf_matrix_t;

// Return a zero vector
pf_vector_t pf_vector_zero(void);

// Simple vector subtraction
pf_vector_t pf_vector_sub(pf_vector_t a, pf_vector_t b);

// Transform from local to global coords (a + b)
pf_vector_t pf_vector_coord_add(pf_vector_t a, pf_vector_t b);

// Transform from global to local coords (a - b)
// pf_vector_t pf_vector_coord_sub(pf_vector_t a, pf_vector_t b);

// Return a zero matrix
pf_matrix_t pf_matrix_zero(void);

// Decompose a covariance matrix [a] into a rotation matrix [r] and a
// diagonal matrix [d] such that a = r * d * r^T.
void pf_matrix_unitary(pf_matrix_t * r, pf_matrix_t * d, pf_matrix_t a);

#ifdef __cplusplus
}
#endif

#endif  // SYNCAI_AMCL__PF__PF_VECTOR_HPP_
