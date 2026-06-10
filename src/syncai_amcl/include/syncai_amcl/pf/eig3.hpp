#ifndef SYNCAI_AMCL__PF__EIG3_HPP_
#define SYNCAI_AMCL__PF__EIG3_HPP_

/* Symmetric matrix A => eigenvectors in columns of V, corresponding
   eigenvalues in d. */
void eigen_decomposition(double A[3][3], double V[3][3], double d[3]);

#endif  // SYNCAI_AMCL__PF__EIG3_HPP_
