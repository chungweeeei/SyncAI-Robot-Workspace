#ifndef SYNCAI_AMCL__PF__PF_PDF_HPP_
#define SYNCAI_AMCL__PF__PF_PDF_HPP_

#include "syncai_amcl/pf/pf_vector.hpp"

// #include <gsl/gsl_rng.h>
// #include <gsl/gsl_randist.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************
 * Gaussian
 *************************************************************************/

// Gaussian PDF info
typedef struct
{
  // Mean, covariance and inverse covariance
  pf_vector_t x;
  pf_matrix_t cx;
  // pf_matrix_t cxi;
  double cxdet;

  // Decomposed covariance matrix (rotation * diagonal)
  pf_matrix_t cr;
  pf_vector_t cd;

  // A random number generator
  // gsl_rng *rng;
} pf_pdf_gaussian_t;

// Create a gaussian pdf
pf_pdf_gaussian_t * pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx);

// Destroy the pdf
void pf_pdf_gaussian_free(pf_pdf_gaussian_t * pdf);

// Compute the value of the pdf at some point [z].
// double pf_pdf_gaussian_value(pf_pdf_gaussian_t *pdf, pf_vector_t z);

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double pf_ran_gaussian(double sigma);

// Generate a sample from the pdf.
pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t * pdf);

#ifdef __cplusplus
}
#endif

#endif  // SYNCAI_AMCL__PF__PF_PDF_HPP_
