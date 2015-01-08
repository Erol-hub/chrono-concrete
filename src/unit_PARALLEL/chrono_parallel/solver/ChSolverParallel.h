// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// This file contains the base class used for all parallel iterative solvers.
// All of the functions are defined here, with the implementation of each solver
// in it's specific cpp file.
// =============================================================================
#ifndef CHSOLVERPARALLEL_H
#define CHSOLVERPARALLEL_H

#include "chrono_parallel/ChBaseParallel.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/constraints/ChConstraintRigidRigid.h"
#include "chrono_parallel/constraints/ChConstraintRigidFluid.h"
#include "chrono_parallel/constraints/ChConstraintFluidFluid.h"
#include "chrono_parallel/constraints/ChConstraintBilateral.h"

namespace chrono {
class CH_PARALLEL_API ChSolverParallel : public ChBaseParallel {
 public:
  ChSolverParallel();
  virtual ~ChSolverParallel();

  // At the beginning of the step reset the size/indexing variables,
  // resize for new contact list and clear temporary accumulation variables
  void Setup(ChParallelDataManager* data_container_    // pointer to data container
             );

  // Project the lagrange multipliers (gamma) onto the friction cone.
  void Project(real* gamma    // Lagrange Multipliers
               );

  // Project a single contact
  void Project_Single(int index,     // index of contact
                      real* gamma    // Lagrange Multipliers
                      );

  // Compute the first half of the shur matrix vector multiplication (N*x)
  // Perform M_invDx=M^-1*D*x
  void shurA(blaze::DynamicVector<real>& x, blaze::DynamicVector<real>& out    // Vector that N is multiplied by
             );

  // Compute rhs value with relaxation term
  void ComputeSRhs(custom_vector<real>& gamma, const custom_vector<real>& rhs, custom_vector<real3>& vel_data, custom_vector<real3>& omg_data, custom_vector<real>& b);

  //   // Perform M^-1*D*gamma and compute the linear and rotational velocity
  //   vectors
  //   void ComputeImpulses(
  //                        custom_vector<real>& gamma,
  //                        custom_vector<real3>& vel_data,
  //                        custom_vector<real3>& omg_data);

  // Function that performs time integration to get the new positions
  // Used when contacts need to be updated within the solver
  // Function is similar to compute impulses
  void UpdatePosition(custom_vector<real>& x    // Lagrange multipliers
                      );

  // Rerun the narrowphase to get the new contact list, broadphase is not run
  // again here
  // This assumes that the positions did not drastically change.
  void UpdateContacts();

  // Compute the full shur matrix vector product (N*x) where N=D^T*M^-1*D
  void ShurProduct(const blaze::DynamicVector<real>& x,    // Vector that will be multiplied by N
                   blaze::DynamicVector<real>& AX          // Output Result
                   );

  // Compute the shur matrix vector product only for the bilaterals (N*x)
  // where N=D^T*M^-1*D
  template <class T>
  void ShurBilaterals(T& x, blaze::DynamicVector<real>& output) {
    blaze::SparseSubmatrix<CompressedMatrix<real> > D_T_sub =
        blaze::submatrix(data_container->host_data.D_T, data_container->num_unilaterals, 0, data_container->num_bilaterals, data_container->num_bodies * 6);

    blaze::SparseSubmatrix<CompressedMatrix<real> > M_invD_sub =
        blaze::submatrix(data_container->host_data.M_invD, 0, data_container->num_unilaterals, data_container->num_bodies * 6, data_container->num_bilaterals);
    output = D_T_sub * M_invD_sub * x;
  }

  // Call this function with an associated solver type to solve the system
  virtual void Solve() = 0;

  // Perform velocity stabilization on bilateral constraints
  uint SolveStab(const uint max_iter,                                     // Maximum number of iterations
                 const uint size,                                         // Number of unknowns
                 const blaze::DenseSubvector<DynamicVector<real> >& b,    // Rhs vector
                 blaze::DenseSubvector<DynamicVector<real> >& x           // The vector of unknowns
                 );

  // Get the number of iterations perfomed by the solver
  int GetIteration() { return current_iteration; }
  // Get the current residual
  real GetResidual() { return residual; }

  real GetObjectiveBlaze(blaze::DynamicVector<real>& x, blaze::DynamicVector<real>& b) {
    blaze::DynamicVector<real> Nl(x.size());
    // f_p = 0.5*l_candidate'*N*l_candidate - l_candidate'*b  =
    // l_candidate'*(0.5*Nl_candidate - b);
    Nl = data_container->host_data.D_T * (data_container->host_data.M_invD * x);    // 1)  g_tmp = N*l_candidate
                                                                                    // ...        ####
                                                                                    // MATR.MULTIPLICATION!!!###
    Nl = 0.5 * Nl - b;                                                              // 2) 0.5*N*l_candidate-b_shur
    return (x, Nl);                                                                 // 3)  mf_p  = l_candidate'*(0.5*N*l_candidate-b_shur)
  }

  real Res4Blaze(blaze::DynamicVector<real>& x, blaze::DynamicVector<real>& b) {
    real gdiff = .1;
    blaze::DynamicVector<real> inside = x - gdiff * (data_container->host_data.D_T * (data_container->host_data.M_invD * x) - b);
    Project(inside.data());
    blaze::DynamicVector<real> temp = (x - inside) / (x.size() * gdiff);
    return sqrt((temp, temp));
  }

  void AtIterationEnd(real maxd, real maxdeltalambda, int iter) {
    maxd_hist.push_back(maxd);
    maxdeltalambda_hist.push_back(maxdeltalambda);
    iter_hist.push_back(iter);
  }

  // Set the maximum number of iterations for all solvers
  void SetMaxIterations(const int max_iteration_value) { max_iteration = max_iteration_value; }

  int current_iteration;    // The current iteration number of the solver
  int max_iteration;        // The maximum number of iterations that the solver will
                            // perform
  int total_iteration;      // The total number of iterations performed, this
                            // variable accumulates
  real residual;            // Current residual for the solver
  real objective_value;

  // These three variables are used to store the convergence history of the
  // solver
  custom_vector<real> maxd_hist, maxdeltalambda_hist, iter_hist;

  ChConstraintRigidRigid* rigid_rigid;
  ChConstraintBilateral* bilateral;
};
}

#endif
