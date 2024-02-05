// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Erol Lale
// =============================================================================
// Builder class for LDPM 
//
//  Builder reads node coordinates, element connectivity, facet info from the file created by preprocessor
//
// 
// =============================================================================

#ifndef CH_BUILDER_LDPM_H
#define CH_BUILDER_LDPM_H

#include "ChElementLDPM.h"
//#include "chrono/fea/ChElementTetraCorot_4.h"


#include "chrono/fea/ChMesh.h"
//#include "chrono/fea/ChElementBeamEuler.h"
//#include "chrono/fea/ChElementBeamIGA.h"
//#include "chrono/fea/ChElementCableANCF.h"
//#include "chrono/fea/ChElementBeamANCF_3333.h"
//#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"
//#include "chrono/fea/ChElementBeamTaperedTimoshenkoFPM.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"

#include "chrono/geometry/ChLineBspline.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

/// Utility class for creating complex beams using ChElementBeamEuler elements, for example subdivides a segment in
/// multiple finite elements.
class ChApi ChBuilderLDPM {
  protected:
    std::vector<std::shared_ptr<ChElementLDPM>> beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>> beam_nodes;

  public:    
    
    void read_LDPM_info(std::shared_ptr<ChMesh> my_mesh,  std::shared_ptr<ChMaterialVECT> vect_mat, std::string& LDPM_data_path, 
				std::string& LDPM_GeoName);

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChElementLDPM>>& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a BuildBeam function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot>>& GetLastBeamNodes() { return beam_nodes; }
};


/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
