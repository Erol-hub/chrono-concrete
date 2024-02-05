// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Erol Lale
// =============================================================================
// Base class for linear tetrahedral FEA elements with rotational degrees of freedom.
// =============================================================================

#ifndef CH_TERAHEDRON_6DOFS_H
#define CH_TERAHEDRON_6DOFS_H

#include "chrono/fea/ChNodeFEAxyzrot.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_elements
/// @{

/// Base class for a FEA element with tetrahedral shape.
class ChApi ChElementTetrahedron_6DOFs {
  public:
    ChElementTetrahedron_6DOFs() {}
    virtual ~ChElementTetrahedron_6DOFs() {}

    /// Return the specified tetrahedron node (0 <= n <= 3).
    virtual std::shared_ptr<ChNodeFEAxyzrot> GetTetrahedronNode(int n) = 0;
};

/// @} fea_elements

}  // namespace fea
}  // namespace chrono

#endif
