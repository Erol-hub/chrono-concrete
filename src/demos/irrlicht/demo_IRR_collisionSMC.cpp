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
// Authors: Radu Serban
// =============================================================================
//
//  Demo code about collisions and contacts using a penalty (SMC) method
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include <irrlicht.h>

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

void AddFallingItems(ChIrrApp& application) {
    // Shared contact material for falling objects
    auto mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    for (int ix = -2; ix < 3; ix++) {
        for (int iz = -2; iz < 3; iz++) {
            // Spheres
            {
                double mass = 1;
                double radius = 1.1;
                auto body = chrono_types::make_shared<ChBody>(ChContactMethod::SMC);
                body->SetInertiaXX((2.0 / 5.0) * mass * pow(radius, 2) * ChVector<>(1, 1, 1));
                body->SetMass(mass);
                body->SetPos(ChVector<>(4.0 * ix, 4.0, 4.0 * iz));

                body->GetCollisionModel()->ClearModel();
                body->GetCollisionModel()->AddSphere(mat, radius);
                body->GetCollisionModel()->BuildModel();
                body->SetCollide(true);

                auto sphere = chrono_types::make_shared<ChSphereShape>();
                sphere->GetSphereGeometry().rad = radius;
                sphere->SetColor(ChColor(0.9f, 0.4f, 0.2f));
                body->AddAsset(sphere);

                application.GetSystem()->AddBody(body);
            }

            // Boxes
            {
                double mass = 1;
                ChVector<> hsize(0.75, 0.75, 0.75);
                auto body = chrono_types::make_shared<ChBody>(ChContactMethod::SMC);

                body->SetMass(mass);
                body->SetPos(ChVector<>(4.0 * ix, 6.0, 4.0 * iz));

                body->GetCollisionModel()->ClearModel();
                body->GetCollisionModel()->AddBox(mat, hsize.x(), hsize.y(), hsize.z());
                body->GetCollisionModel()->BuildModel();
                body->SetCollide(true);

                auto box = chrono_types::make_shared<ChBoxShape>();
                box->GetBoxGeometry().Size = hsize;
                box->SetColor(ChColor(0.4f, 0.9f, 0.2f));
                body->AddAsset(box);

                application.GetSystem()->AddBody(body);
            }
        }
    }
}

void AddContainerWall(std::shared_ptr<ChBody> body,
                      std::shared_ptr<ChMaterialSurface> mat,
                      const ChVector<>& size,
                      const ChVector<>& pos,
                      bool visible = true) {
    ChVector<> hsize = 0.5 * size;

    body->GetCollisionModel()->AddBox(mat, hsize.x(), hsize.y(), hsize.z(), pos);

    if (visible) {
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Pos = pos;
        box->GetBoxGeometry().Size = hsize;
        box->SetColor(ChColor(1, 0, 0));
        box->SetFading(0.6f);
        body->AddAsset(box);
    }
}

void AddContainer(ChIrrApp& application) {
    // The fixed body (5 walls)
    auto fixedBody = chrono_types::make_shared<ChBody>(ChContactMethod::SMC);

    fixedBody->SetMass(1.0);
    fixedBody->SetBodyFixed(true);
    fixedBody->SetPos(ChVector<>());
    fixedBody->SetCollide(true);

    // Contact material for container
    auto fixed_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    fixedBody->GetCollisionModel()->ClearModel();
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(20, 1, 20), ChVector<>(0, -5, 0));
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(1, 10, 20.99), ChVector<>(-10, 0, 0));
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(1, 10, 20.99), ChVector<>(10, 0, 0));
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(20.99, 10, 1), ChVector<>(0, 0, -10), false);
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(20.99, 10, 1), ChVector<>(0, 0, 10));
    fixedBody->GetCollisionModel()->BuildModel();

    application.GetSystem()->AddBody(fixedBody);

    // The rotating mixer body
    auto rotatingBody = chrono_types::make_shared<ChBody>(ChContactMethod::SMC);

    rotatingBody->SetMass(10.0);
    rotatingBody->SetInertiaXX(ChVector<>(50, 50, 50));
    rotatingBody->SetPos(ChVector<>(0, -1.6, 0));
    rotatingBody->SetCollide(true);

    // Contact material for mixer body
    auto rot_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    ChVector<> hsize(5, 2.75, 0.5);

    rotatingBody->GetCollisionModel()->ClearModel();
    rotatingBody->GetCollisionModel()->AddBox(rot_mat, hsize.x(), hsize.y(), hsize.z());
    rotatingBody->GetCollisionModel()->BuildModel();

    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = hsize;
    box->SetColor(ChColor(0, 0, 1));
    box->SetFading(0.6f);
    rotatingBody->AddAsset(box);

    application.GetSystem()->AddBody(rotatingBody);

    // A motor between the two
    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();

    my_motor->Initialize(rotatingBody, fixedBody, ChFrame<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X)));
    auto mfun = chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 2.0);  // speed w=90�/s
    my_motor->SetSpeedFunction(mfun);

    application.GetSystem()->AddLink(my_motor);
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Simulation and rendering time-step
    double time_step = 1e-4;
    double out_step = 0.02;

    // Create a ChronoENGINE physical system
    ChSystemSMC mphysicalSystem;
    mphysicalSystem.Set_G_acc(0.38 * mphysicalSystem.Get_G_acc());

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"SMC collision demo", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 18, -20));

    // Add fixed and moving bodies
    AddContainer(application);
    AddFallingItems(application);

    // Complete asset specification: convert all assets to Irrlicht
    application.AssetBindAll();
    application.AssetUpdateAll();

    // The soft-real-time cycle
    double time = 0;
    double out_time = 0;
    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        while (time < out_time) {
            mphysicalSystem.DoStepDynamics(time_step);
            time += time_step;
        }
        out_time += out_step;

        application.EndScene();
    }

    return 0;
}
