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
// Authors: Mariusz Warzecha
// =============================================================================
//
//  Application created to test DFC model implementation in chrono::multicore

#include "chrono_multicore/physics/ChSystemMulticore.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactSMC.h"
#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/assets/ChSphereShape.h"

#include "chrono/collision/ChCollisionSystem.h"
#include "chrono/collision/ChCollisionSystemBullet.h" 
 

#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/stringbuffer.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/assets/ChVisualSystem.h"


using namespace chrono;
using namespace chrono::irrlicht;
chrono::collision::ChCollisionSystemType collision_type = chrono::collision::ChCollisionSystemType::BULLET;

class MyContactReport : public chrono::ChContactContainer::ReportContactCallback {
    struct CollisionData {
        ChVector<> pA;             ///< contact pA
        ChVector<> pB;             ///< contact pB
        ChMatrix33<> plane_coord;  ///< contact plane coordsystem (A column 'X' is contact normal)
        double distance;                   ///< contact distance
        double eff_radius;                 ///< effective radius of curvature at contact
        ChVector<> react_forces;   ///< react.forces (if already computed). In coordsystem 'plane_coord'
        ChVector<> react_torques;  ///< react.torques, if rolling friction (if already computed).
        ChContactable* contactobjA;  ///< model A (note: some containers may not support it and could be nullptr)
        ChContactable* contactobjB;  ///< model B (note: some containers may not support it and could be nullptr)
        CollisionData(
            const ChVector<>& fpA,             ///< contact pA
            const ChVector<>& fpB,             ///< contact pB
            const ChMatrix33<>& fplane_coord,  ///< contact plane coordsystem (A column 'X' is contact normal)
            const double& fdistance,                   ///< contact distance
            const double& feff_radius,                 ///< effective radius of curvature at contact
            const ChVector<>& freact_forces,  ///< react.forces (if already computed). In coordsystem 'plane_coord'
            const ChVector<>& freact_torques,  ///< react.torques, if rolling friction (if already computed).
            ChContactable* fcontactobjA,  ///< model A (note: some containers may not support it and could be nullptr)
            ChContactable* fcontactobjB  ///< model B (note: some containers may not support it and could be nullptr)
        ) {
            pA = fpA;
            pB = fpB;
            plane_coord = fplane_coord;
            distance = fdistance;
            eff_radius = feff_radius;
            react_forces = freact_forces;
            react_torques = freact_forces;
            contactobjA = fcontactobjA;
            contactobjB = fcontactobjB;
        };
    };

  public:
    std::vector<CollisionData> VectorOfCollisionData;

    bool OnReportContact(
        const ChVector<>& pA,             ///< contact pA
        const ChVector<>& pB,             ///< contact pB
        const ChMatrix33<>& plane_coord,  ///< contact plane coordsystem (A column 'X' is contact normal)
        const double& distance,           ///< contact distance
        const double& eff_radius,         ///< effective radius of curvature at contact
        const ChVector<>& react_forces,   ///< react.forces (if already computed). In coordsystem 'plane_coord'
        const ChVector<>& react_torques,  ///< react.torques, if rolling friction (if already computed).
        ChContactable* contactobjA,       ///< model A (note: some containers may not support it and could be nullptr)
        ChContactable* contactobjB  ///< model B (note: some containers may not support it and could be nullptr)
        ) override {
        CollisionData TempCollisionData(pA, pB, plane_coord, distance, eff_radius, react_forces, react_torques,
                                        contactobjA, contactobjB);
        VectorOfCollisionData.push_back(TempCollisionData);
        return 1;
    };
};


std::vector<std::shared_ptr<ChBody>> AddBalls(ChSystemMulticore& sys, int ball_number, double initial_velocity) {
    std::vector<std::shared_ptr<ChBody>> collection_of_balls;
    // Define material for balls, it holds Young modulus, Poisson ratio, etc.
    // standard constructor (without arguments) has data for steel, i.e. Young modulus = 2e5
    auto mat = chrono_types::make_shared<chrono::ChMaterialSurfaceSMC>();
    mat->SetYoungModulus(2.05e11);
    mat->SetPoissonRatio(0.3);
    mat->SetGn(0);
    mat->SetGt(0);

    // Collision envelope
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.004);

    double gap_between_sphere_1_2 = 0.000;

    // Define balls
    double radius = 0.01;                                          // in meters, I have doubt about used units
    double mass = ((4.0 / 3.0) * 3.1415 * pow(radius, 3)) * 1000;  // calculated (volume, density), in kg, average
    for (int i = 0; i < ball_number; i++) {
        //auto ball = chrono_types::make_shared<ChBody>();
        auto ball = std::shared_ptr<chrono::ChBody>(sys.NewBody());
        // ChVector is a template class, by default (if nothing in <>) it is double
        ball->SetInertiaXX((2.0 / 5.0) * mass * pow(radius, 3) * chrono::ChVector<>(1, 1, 1));
        ball->SetMass(mass);
        if (i == 0) {
            ball->SetPos(chrono::ChVector<>(0, 0, 0));
            ball->SetPos_dt(chrono::ChVector<>(initial_velocity, 0, 0));
        } else {
            ball->SetPos(chrono::ChVector<>(i *( 2 * radius - radius/2 ) + gap_between_sphere_1_2, 0, 0));
            ball->SetPos_dt(chrono::ChVector<>(0, 0, 0));
        }
        ball->GetCollisionModel()->ClearModel();  // CollisionModel is a class which defines the geometric model used
                                                  // for collision detection (it has simple shapes and mesh)
        ball->GetCollisionModel()->AddSphere(mat, radius);
        ball->SetBodyFixed(false);
        ball->SetCollide(true);
        ball->GetCollisionModel()->BuildModel();
        auto sphere1 = chrono_types::make_shared<ChSphereShape>(radius);
        sphere1->SetTexture(GetChronoDataFile("textures/bluewhite.png"));
        sphere1->SetOpacity(0.4f);
        auto sphere2 = chrono_types::make_shared<ChSphereShape>(radius/2);
        sphere2->SetTexture(GetChronoDataFile("textures/rock.png"));
        auto ball_vis = chrono_types::make_shared<ChVisualModel>();
        ball_vis->AddShape(sphere1);
        ball_vis->AddShape(sphere2);
        ball->AddVisualModel(ball_vis);
        sys.AddBody(ball);
        collection_of_balls.push_back(ball);
    }
    return collection_of_balls;
}


int main(int argc, char* argv[]) {
    GetLog() << "Test application for verification the DFC contact force model in chrono::multicore\n";
    GetLog() << "Based on open source library projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

	// Set path to Chrono data directory
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);

    // Simulation and rendering time-step
    double time_step = 2e-4;
    double out_step = 2e-2;

	// Create the physical system, argument use_material_properties = true defines how parameters
    // of the contact model are calcultated (true means that Young modulus and Poisson ratio
    // are used to calculate kn, not relevant in this case as they will be determined by contact force implementation

    ChSystemMulticoreSMC sys;  // ChSystemSMC is a class, create object of this class, this system is using
                                    // material properties to calculate contact force parameters
    //double time_step = 1e-6;
    int frame_skip = 100;
    uint max_iteration = 100;
    chrono::real tolerance = 1e-3;

    sys.Set_G_acc(chrono::ChVector<>(0, 0, 0));

    // Set solver parameters
    sys.GetSettings()->solver.max_iteration_bilateral = max_iteration;
    sys.GetSettings()->solver.tolerance = tolerance;

    sys.GetSettings()->collision.narrowphase_algorithm = chrono::collision::ChNarrowphase::Algorithm::HYBRID;
    sys.GetSettings()->collision.bins_per_axis = chrono::vec3(10, 10, 10);

    // The following two lines are optional, since they are the default options. They are added for future reference,
    // i.e. when needed to change those models.
    sys.GetSettings()->solver.contact_force_model = chrono::ChSystemSMC::ContactForceModel::DFC;
  
    int ball_quantity = 4;
    auto created_balls = AddBalls(sys,ball_quantity , 0.01);
    
	std::shared_ptr<ChVisualSystem> vis;
	auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis_irr->AttachSystem(&sys);
    vis_irr->SetWindowSize(800, 600);
    vis_irr->SetWindowTitle("SMC callbacks");
    vis_irr->Initialize();
    vis_irr->AddLogo();
    vis_irr->AddSkyBox();
    vis_irr->AddCamera(ChVector<>(0, 0.05, -0.1));
    vis_irr->AddTypicalLights();
    vis = vis_irr;

    std::ofstream data_files[4];
    for (int k = 1; k < ball_quantity + 1; k++) {
        std::string file_name =
            std::to_string(ball_quantity) + "_" + std::to_string(1) + "_ball_" + std::to_string(k) + ".txt";
        const char* file_name_to_pass = file_name.c_str();  // convert type for object constructor
        data_files[k - 1].open(file_name_to_pass);
        data_files[k - 1] << "Simulation time [s]"
                          << ", "
                          << "Position [m]"
                          << ", "
                          << "Velocity [m/s]"
                          << ", "
                          << "Acceleration [m/s^2]"
                          << ", "
                          << "Contact force resultant [N]"
                          << ", "
                          << "Contact deformation [m]"
                          << "\n";
    }
	// a file to store evolution of strain data during collision
    std::ofstream strain_data;
    strain_data.open("all_strain_data.txt");

    double simulation_time = 0;
    while (simulation_time <= 4) {
        vis->BeginScene();
        vis->Render();
        vis->RenderGrid(ChFrame<>(VNULL, Q_from_AngX(CH_C_PI_2)), 12, 0.5);
        vis->RenderCOGFrames(1.0);
        sys.DoStepDynamics(time_step);
        vis->EndScene();
        simulation_time += time_step;
        std::shared_ptr<MyContactReport> PointerToContactDataStoringClass = std::make_shared<MyContactReport>();
        sys.GetContactContainer()->ReportAllContacts(PointerToContactDataStoringClass);
        for (int k = 0; k < ball_quantity; k++) {
            data_files[k] << simulation_time << ", " << created_balls[k]->GetPos()[0] << ", "
                          << created_balls[k]->GetPos_dt()[0] << ", " << created_balls[k]->GetPos_dtdt()[0] << ", ";
            //<< created_balls[k]->GetContactForce()[0] << ", ";
            if (!PointerToContactDataStoringClass->VectorOfCollisionData.empty()) {
                bool added_contact_force = false;
                for (auto e : PointerToContactDataStoringClass->VectorOfCollisionData) {
                    // chrono::GetLog() << e.pA << " Point A for iteration: " << k << e.react_forces << "Vector size: "
                    // << PointerToContactDataStoringClass->VectorOfCollisionData.size() << "\n"; chrono::GetLog() <<
                    // "Simulation time" << simulation_time << "\n"; chrono::GetLog() << "Configuration number: " << i
                    // << "_" << j << "\n";
                    if (e.contactobjA == created_balls[k]->GetCollisionModel()->GetContactable()) {
                        data_files[k] << e.react_forces[0] << ", " << e.distance << "\n";
                        added_contact_force = true;
                    }
                }
                if (!added_contact_force) {
                    data_files[k] << "0, " << "0\n";
                }
            } else {
                data_files[k] << "0, " << "0\n";
            }
        }
    }
    strain_data.close();
    return 0;
 }