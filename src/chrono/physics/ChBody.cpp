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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include <cstdlib>
#include <algorithm>

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChTransform.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/physics/ChMarker.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

using namespace geometry;

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChBody)
CH_UPCASTING(ChBody, ChPhysicsItem)
CH_UPCASTING(ChBody, ChBodyFrame)
CH_UPCASTING_SANITIZED(ChBody, ChContactable_1vars<6>, ChBody_ChContactable_1vars_6)
CH_UPCASTING(ChBody, ChLoadableUVW)

ChBody::ChBody()
    : body_id(0), fixed(false), collide(false), Xforce(VNULL), Xtorque(VNULL), Force_acc(VNULL), Torque_acc(VNULL) {
    marklist.clear();
    forcelist.clear();

    BFlagsSetAllOFF();  // no flags

    density = 1000.0f;

    max_speed = 0.5f;
    max_wvel = 2.0f * float(CH_C_PI);

    sleep_time = 0.6f;
    sleep_starttime = 0;
    sleep_minspeed = 0.1f;
    sleep_minwvel = 0.04f;
    SetUseSleeping(true);

    variables.SetUserData((void*)this);
}

ChBody::ChBody(const ChBody& other) : ChPhysicsItem(other), ChBodyFrame(other) {
    fixed = other.fixed;
    collide = other.collide;
    bflags = other.bflags;

    variables = other.variables;
    variables.SetUserData((void*)this);

    gyro = other.gyro;

    RemoveAllForces();   // also copy-duplicate the forces? Let the user handle this..
    RemoveAllMarkers();  // also copy-duplicate the markers? Let the user handle this..

    ChTime = other.ChTime;

    // Copy the collision model if any
    if (other.collision_model) {
        collision_model = chrono_types::make_shared<ChCollisionModel>(*other.collision_model);
        collision_model->SetContactable(this);
    }

    density = other.density;

    max_speed = other.max_speed;
    max_wvel = other.max_wvel;

    sleep_time = other.sleep_time;
    sleep_starttime = other.sleep_starttime;
    sleep_minspeed = other.sleep_minspeed;
    sleep_minwvel = other.sleep_minwvel;
}

ChBody::~ChBody() {
    RemoveAllForces();
    RemoveAllMarkers();
}

//// STATE BOOKKEEPING FUNCTIONS

void ChBody::IntStateGather(const unsigned int off_x,  // offset in x state vector
                            ChState& x,                // state vector, position part
                            const unsigned int off_v,  // offset in v state vector
                            ChStateDelta& v,           // state vector, speed part
                            double& T                  // time
) {
    x.segment(off_x + 0, 3) = coord.pos.eigen();
    x.segment(off_x + 3, 4) = coord.rot.eigen();
    v.segment(off_v + 0, 3) = coord_dt.pos.eigen();
    v.segment(off_v + 3, 3) = GetWvel_loc().eigen();
    T = GetChTime();
}

void ChBody::IntStateScatter(const unsigned int off_x,  // offset in x state vector
                             const ChState& x,          // state vector, position part
                             const unsigned int off_v,  // offset in v state vector
                             const ChStateDelta& v,     // state vector, speed part
                             const double T,            // time
                             bool full_update           // perform complete update
) {
    SetCoord(x.segment(off_x, 7));
    SetPos_dt(v.segment(off_v + 0, 3));
    SetWvel_loc(v.segment(off_v + 3, 3));
    SetChTime(T);
    Update(T, full_update);
}

void ChBody::IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) {
    a.segment(off_a + 0, 3) = coord_dtdt.pos.eigen();
    a.segment(off_a + 3, 3) = GetWacc_loc().eigen();
}

void ChBody::IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) {
    SetPos_dtdt(a.segment(off_a + 0, 3));
    SetWacc_loc(a.segment(off_a + 3, 3));
}

void ChBody::IntStateIncrement(const unsigned int off_x,  // offset in x state vector
                               ChState& x_new,            // state vector, position part, incremented result
                               const ChState& x,          // state vector, initial position part
                               const unsigned int off_v,  // offset in v state vector
                               const ChStateDelta& Dv     // state vector, increment
) {
    // ADVANCE POSITION:
    x_new(off_x) = x(off_x) + Dv(off_v);
    x_new(off_x + 1) = x(off_x + 1) + Dv(off_v + 1);
    x_new(off_x + 2) = x(off_x + 2) + Dv(off_v + 2);

    // ADVANCE ROTATION: R_new = DR_a * R_old
    // (using quaternions, local or abs:  q_new = Dq_a * q_old =  q_old * Dq_l  )
    ChQuaternion<> q_old(x.segment(off_x + 3, 4));
    ChQuaternion<> rel_q;
    rel_q.Q_from_Rotv(Dv.segment(off_v + 3, 3));
    ChQuaternion<> q_new = q_old * rel_q;
    x_new.segment(off_x + 3, 4) = q_new.eigen();
}

void ChBody::IntStateGetIncrement(const unsigned int off_x,  // offset in x state vector
                                  const ChState& x_new,      // state vector, position part, incremented result
                                  const ChState& x,          // state vector, initial position part
                                  const unsigned int off_v,  // offset in v state vector
                                  ChStateDelta& Dv           // state vector, increment
) {
    // POSITION:
    Dv(off_v) = x_new(off_x) - x(off_x);
    Dv(off_v + 1) = x_new(off_x + 1) - x(off_x + 1);
    Dv(off_v + 2) = x_new(off_x + 2) - x(off_x + 2);

    // ROTATION (quaternions): Dq_loc = q_old^-1 * q_new,
    //  because   q_new = Dq_abs * q_old   = q_old * Dq_loc
    ChQuaternion<> q_old(x.segment(off_x + 3, 4));
    ChQuaternion<> q_new(x_new.segment(off_x + 3, 4));
    ChQuaternion<> rel_q = q_old.GetConjugate() % q_new;
    Dv.segment(off_v + 3, 3) = rel_q.Q_to_Rotv().eigen();
}

void ChBody::IntLoadResidual_F(const unsigned int off,  // offset in R residual
                               ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                               const double c           // a scaling factor
) {
    // add applied forces to 'fb' vector
    R.segment(off, 3) += c * Xforce.eigen();

    // add applied torques to 'fb' vector, including gyroscopic torque
    if (GetNoGyroTorque())
        R.segment(off + 3, 3) += c * Xtorque.eigen();
    else
        R.segment(off + 3, 3) += c * (Xtorque - gyro).eigen();
}

void ChBody::IntLoadResidual_Mv(const unsigned int off,      // offset in R residual
                                ChVectorDynamic<>& R,        // result: the R residual, R += c*M*v
                                const ChVectorDynamic<>& w,  // the w vector
                                const double c               // a scaling factor
) {
    R(off + 0) += c * GetMass() * w(off + 0);
    R(off + 1) += c * GetMass() * w(off + 1);
    R(off + 2) += c * GetMass() * w(off + 2);
    ChVector<> Iw = GetInertia() * ChVector<>(w.segment(off + 3, 3));
    Iw *= c;
    R.segment(off + 3, 3) += Iw.eigen();
}

void ChBody::IntToDescriptor(const unsigned int off_v,
                             const ChStateDelta& v,
                             const ChVectorDynamic<>& R,
                             const unsigned int off_L,
                             const ChVectorDynamic<>& L,
                             const ChVectorDynamic<>& Qc) {
    variables.Get_qb() = v.segment(off_v, 6);
    variables.Get_fb() = R.segment(off_v, 6);
}

void ChBody::IntFromDescriptor(const unsigned int off_v,  // offset in v
                               ChStateDelta& v,
                               const unsigned int off_L,  // offset in L
                               ChVectorDynamic<>& L) {
    v.segment(off_v, 6) = variables.Get_qb();
}

////

void ChBody::InjectVariables(ChSystemDescriptor& mdescriptor) {
    variables.SetDisabled(!IsActive());

    mdescriptor.InsertVariables(&variables);
}

void ChBody::VariablesFbReset() {
    variables.Get_fb().setZero();
}

void ChBody::VariablesFbLoadForces(double factor) {
    // add applied forces to 'fb' vector
    variables.Get_fb().segment(0, 3) += factor * Xforce.eigen();

    // add applied torques to 'fb' vector, including gyroscopic torque
    if (GetNoGyroTorque())
        variables.Get_fb().segment(3, 3) += factor * Xtorque.eigen();
    else
        variables.Get_fb().segment(3, 3) += factor * (Xtorque - gyro).eigen();
}

void ChBody::VariablesFbIncrementMq() {
    variables.Compute_inc_Mb_v(variables.Get_fb(), variables.Get_qb());
}

void ChBody::VariablesQbLoadSpeed() {
    // set current speed in 'qb', it can be used by the solver when working in incremental mode
    variables.Get_qb().segment(0, 3) = GetCoord_dt().pos.eigen();
    variables.Get_qb().segment(3, 3) = GetWvel_loc().eigen();
}

void ChBody::VariablesQbSetSpeed(double step) {
    ChCoordsys<> old_coord_dt = GetCoord_dt();

    // from 'qb' vector, sets body speed, and updates auxiliary data
    SetPos_dt(variables.Get_qb().segment(0, 3));
    SetWvel_loc(variables.Get_qb().segment(3, 3));

    // apply limits (if in speed clamping mode) to speeds.
    ClampSpeed();

    // compute auxiliary gyroscopic forces
    ComputeGyro();

    // Compute accel. by BDF (approximate by differentiation);
    if (step) {
        SetPos_dtdt((GetCoord_dt().pos - old_coord_dt.pos) / step);
        SetRot_dtdt((GetCoord_dt().rot - old_coord_dt.rot) / step);
    }
}

void ChBody::VariablesQbIncrementPosition(double dt_step) {
    if (!IsActive())
        return;

    // Updates position with incremental action of speed contained in the
    // 'qb' vector:  pos' = pos + dt * speed   , like in an Eulero step.

    ChVector<> newspeed(variables.Get_qb().segment(0, 3));
    ChVector<> newwel(variables.Get_qb().segment(3, 3));

    // ADVANCE POSITION: pos' = pos + dt * vel
    SetPos(GetPos() + newspeed * dt_step);

    // ADVANCE ROTATION: rot' = [dt*wwel]%rot  (use quaternion for delta rotation)
    ChQuaternion<> mdeltarot;
    ChQuaternion<> moldrot = GetRot();
    ChVector<> newwel_abs = Amatrix * newwel;
    double mangle = newwel_abs.Length() * dt_step;
    newwel_abs.Normalize();
    mdeltarot.Q_from_AngAxis(mangle, newwel_abs);
    ChQuaternion<> mnewrot = mdeltarot % moldrot;
    SetRot(mnewrot);
}

void ChBody::SetNoSpeedNoAcceleration() {
    SetPos_dt(VNULL);
    SetWvel_loc(VNULL);
    SetPos_dtdt(VNULL);
    SetRot_dtdt(QNULL);
}

////
void ChBody::ClampSpeed() {
    if (GetLimitSpeed()) {
        double w = 2.0 * coord_dt.rot.Length();
        if (w > max_wvel)
            coord_dt.rot *= max_wvel / w;

        double v = coord_dt.pos.Length();
        if (v > max_speed)
            coord_dt.pos *= max_speed / v;
    }
}

//// Utilities for coordinate transformations

ChVector<> ChBody::Point_World2Body(const ChVector<>& mpoint) {
    return ChFrame<double>::TransformParentToLocal(mpoint);
}

ChVector<> ChBody::Point_Body2World(const ChVector<>& mpoint) {
    return ChFrame<double>::TransformLocalToParent(mpoint);
}

ChVector<> ChBody::Dir_World2Body(const ChVector<>& dir) {
    return Amatrix.transpose() * dir;
}

ChVector<> ChBody::Dir_Body2World(const ChVector<>& dir) {
    return Amatrix * dir;
}

ChVector<> ChBody::RelPoint_AbsSpeed(const ChVector<>& mrelpoint) {
    return PointSpeedLocalToParent(mrelpoint);
}

ChVector<> ChBody::RelPoint_AbsAcc(const ChVector<>& mrelpoint) {
    return PointAccelerationLocalToParent(mrelpoint);
}

// The inertia tensor functions

void ChBody::SetInertia(const ChMatrix33<>& newXInertia) {
    variables.SetBodyInertia(newXInertia);
}

void ChBody::SetInertiaXX(const ChVector<>& iner) {
    variables.GetBodyInertia()(0, 0) = iner.x();
    variables.GetBodyInertia()(1, 1) = iner.y();
    variables.GetBodyInertia()(2, 2) = iner.z();
    variables.GetBodyInvInertia() = variables.GetBodyInertia().inverse();
}

void ChBody::SetInertiaXY(const ChVector<>& iner) {
    variables.GetBodyInertia()(0, 1) = iner.x();
    variables.GetBodyInertia()(0, 2) = iner.y();
    variables.GetBodyInertia()(1, 2) = iner.z();
    variables.GetBodyInertia()(1, 0) = iner.x();
    variables.GetBodyInertia()(2, 0) = iner.y();
    variables.GetBodyInertia()(2, 1) = iner.z();
    variables.GetBodyInvInertia() = variables.GetBodyInertia().inverse();
}

ChVector<> ChBody::GetInertiaXX() const {
    ChVector<> iner;
    iner.x() = variables.GetBodyInertia()(0, 0);
    iner.y() = variables.GetBodyInertia()(1, 1);
    iner.z() = variables.GetBodyInertia()(2, 2);
    return iner;
}

ChVector<> ChBody::GetInertiaXY() const {
    ChVector<> iner;
    iner.x() = variables.GetBodyInertia()(0, 1);
    iner.y() = variables.GetBodyInertia()(0, 2);
    iner.z() = variables.GetBodyInertia()(1, 2);
    return iner;
}

void ChBody::ComputeQInertia(ChMatrix44<>& mQInertia) {
    // [Iq]=[G'][Ix][G]
    ChGlMatrix34<> Gl(coord.rot);
    mQInertia = Gl.transpose() * GetInertia() * Gl;
}

//////

void ChBody::Empty_forces_accumulators() {
    Force_acc = VNULL;
    Torque_acc = VNULL;
}

void ChBody::Accumulate_force(const ChVector<>& force, const ChVector<>& appl_point, bool local) {
    ChVector<> absforce;
    ChVector<> abstorque;
    To_abs_forcetorque(force, appl_point, local, absforce, abstorque);

    Force_acc += absforce;
    Torque_acc += Dir_World2Body(abstorque);
}

void ChBody::Accumulate_torque(const ChVector<>& torque, bool local) {
    if (local) {
        Torque_acc += torque;
    } else {
        Torque_acc += Dir_World2Body(torque);
    }
}

//////

void ChBody::ComputeGyro() {
    ChVector<> Wvel = GetWvel_loc();
    gyro = Vcross(Wvel, variables.GetBodyInertia() * Wvel);
}

bool ChBody::TrySleeping() {
    BFlagSet(BodyFlag::COULDSLEEP, false);

    if (GetUseSleeping()) {
        if (!IsActive())
            return false;

        // if not yet sleeping:
        if ((coord_dt.pos.LengthInf() < sleep_minspeed) && (2.0 * coord_dt.rot.LengthInf() < sleep_minwvel)) {
            if ((GetChTime() - sleep_starttime) > sleep_time) {
                BFlagSet(BodyFlag::COULDSLEEP, true);  // mark as sleep candidate
                return true;                           // could go to sleep!
            }
        } else {
            sleep_starttime = float(GetChTime());
        }
    }
    return false;
}

void ChBody::AddMarker(std::shared_ptr<ChMarker> amarker) {
    // don't allow double insertion of same object
    assert(std::find<std::vector<std::shared_ptr<ChMarker>>::iterator>(marklist.begin(), marklist.end(), amarker) ==
           marklist.end());

    amarker->SetBody(this);
    marklist.push_back(amarker);

    // If the body is already added to a system, mark the system uninitialized and out-of-date
    if (system) {
        system->is_initialized = false;
        system->is_updated = false;
    }
}

void ChBody::AddForce(std::shared_ptr<ChForce> aforce) {
    // don't allow double insertion of same object
    assert(std::find<std::vector<std::shared_ptr<ChForce>>::iterator>(forcelist.begin(), forcelist.end(), aforce) ==
           forcelist.end());

    aforce->SetBody(this);
    forcelist.push_back(aforce);

    // If the body is already added to a system, mark the system uninitialized and out-of-date
    if (system) {
        system->is_initialized = false;
        system->is_updated = false;
    }
}

void ChBody::RemoveForce(std::shared_ptr<ChForce> mforce) {
    // trying to remove objects not previously added?
    assert(std::find<std::vector<std::shared_ptr<ChForce>>::iterator>(forcelist.begin(), forcelist.end(), mforce) !=
           forcelist.end());

    // warning! linear time search
    forcelist.erase(
        std::find<std::vector<std::shared_ptr<ChForce>>::iterator>(forcelist.begin(), forcelist.end(), mforce));

    mforce->SetBody(0);

    // If the body is already added to a system, mark the system out-of-date
    if (system) {
        system->is_updated = false;
    }
}

void ChBody::RemoveMarker(std::shared_ptr<ChMarker> mmarker) {
    // trying to remove objects not previously added?
    assert(std::find<std::vector<std::shared_ptr<ChMarker>>::iterator>(marklist.begin(), marklist.end(), mmarker) !=
           marklist.end());

    // warning! linear time search
    marklist.erase(
        std::find<std::vector<std::shared_ptr<ChMarker>>::iterator>(marklist.begin(), marklist.end(), mmarker));

    mmarker->SetBody(0);

    // If the body is already added to a system, mark the system out-of-date
    if (system) {
        system->is_updated = false;
    }
}

void ChBody::RemoveAllForces() {
    for (auto& force : forcelist) {
        force->SetBody(NULL);
    }
    forcelist.clear();
}

void ChBody::RemoveAllMarkers() {
    for (auto& marker : marklist) {
        marker->SetBody(NULL);
    }
    marklist.clear();
}

std::shared_ptr<ChMarker> ChBody::SearchMarker(const std::string& name) const {
    auto marker = std::find_if(std::begin(marklist), std::end(marklist),
                               [name](std::shared_ptr<ChMarker> marker) { return marker->GetNameString() == name; });
    return (marker != std::end(marklist)) ? *marker : nullptr;
}

std::shared_ptr<ChMarker> ChBody::SearchMarker(int id) const {
    auto marker = std::find_if(std::begin(marklist), std::end(marklist),
                               [id](std::shared_ptr<ChMarker> marker) { return marker->GetIdentifier() == id; });
    return (marker != std::end(marklist)) ? *marker : nullptr;
}

std::shared_ptr<ChForce> ChBody::SearchForce(const std::string& name) const {
    auto force = std::find_if(std::begin(forcelist), std::end(forcelist),
                              [name](std::shared_ptr<ChForce> force) { return force->GetNameString() == name; });
    return (force != std::end(forcelist)) ? *force : nullptr;
}

// These are the members used to UPDATE the body coordinates during the animation.
// Also the coordinates of forces and markers linked to the body will be updated.

void ChBody::UpdateMarkers(double mytime) {
    for (auto& marker : marklist) {
        marker->Update(mytime);
    }
}

void ChBody::UpdateForces(double mytime) {
    // Initialize body force (in abs. coords) and torque (in local coords)
    // with current values from the accumulators.
    Xforce = Force_acc;
    Xtorque = Torque_acc;

    // Accumulate other applied forces
    ChVector<> mforce;
    ChVector<> mtorque;

    for (auto& force : forcelist) {
        // update positions, f=f(t,q)
        force->Update(mytime);

        force->GetBodyForceTorque(mforce, mtorque);
        Xforce += mforce;
        Xtorque += mtorque;
    }

    // Add gravitational forces
    if (system) {
        Xforce += system->Get_G_acc() * GetMass();
    }
}

void ChBody::UpdateTime(double mytime) {
    ChTime = mytime;
}

// UpdateALL updates the state and time
// of the object AND the dependant (linked)
// markers and forces.

void ChBody::Update(bool update_assets) {
    // TrySleeping();         // See if the body can fall asleep; if so, put it to sleeping
    ClampSpeed();   // Apply limits (if in speed clamping mode) to speeds.
    ComputeGyro();  // Set the gyroscopic momentum.

    // Also update the children "markers" and
    // "forces" depending on the body current state.
    UpdateMarkers(ChTime);

    UpdateForces(ChTime);

    // This will update assets
    ChPhysicsItem::Update(ChTime, update_assets);
}

// As before, but keeps the current state.
// Mostly used for world reference body.
void ChBody::Update(double mytime, bool update_assets) {
    // For the body:
    UpdateTime(mytime);

    Update(update_assets);
}

// ---------------------------------------------------------------------------
// Body flags management
void ChBody::BFlagsSetAllOFF() {
    bflags = 0;
}
void ChBody::BFlagsSetAllON() {
    bflags = 0;
    bflags = ~bflags;
}
void ChBody::BFlagSetON(BodyFlag mask) {
    bflags |= mask;
}
void ChBody::BFlagSetOFF(BodyFlag mask) {
    bflags &= ~mask;
}
bool ChBody::BFlagGet(BodyFlag mask) const {
    return (bflags & mask) != 0;
};
void ChBody::BFlagSet(BodyFlag mask, bool state) {
    if (state)
        bflags |= mask;
    else
        bflags &= ~mask;
}

void ChBody::SetBodyFixed(bool state) {
    variables.SetDisabled(state);
    fixed = state;
}

bool ChBody::GetBodyFixed() const {
    return fixed;
}

void ChBody::SetLimitSpeed(bool state) {
    BFlagSet(BodyFlag::LIMITSPEED, state);
}

bool ChBody::GetLimitSpeed() const {
    return BFlagGet(BodyFlag::LIMITSPEED);
}

void ChBody::SetNoGyroTorque(bool state) {
    BFlagSet(BodyFlag::NOGYROTORQUE, state);
}

bool ChBody::GetNoGyroTorque() const {
    return BFlagGet(BodyFlag::NOGYROTORQUE);
}

void ChBody::SetUseSleeping(bool state) {
    BFlagSet(BodyFlag::USESLEEPING, state);
}

bool ChBody::GetUseSleeping() const {
    return BFlagGet(BodyFlag::USESLEEPING);
}

void ChBody::SetSleeping(bool state) {
    BFlagSet(BodyFlag::SLEEPING, state);
}

bool ChBody::GetSleeping() const {
    return BFlagGet(BodyFlag::SLEEPING);
}

bool ChBody::IsActive() const {
    return !BFlagGet(BodyFlag::SLEEPING) && !fixed;
}

// ---------------------------------------------------------------------------
// Collision-related functions

void ChBody::SetCollide(bool state) {
    // Nothing to do if no change in state
    if (state == collide)
        return;

    collide = state;

    // Nothing to do if body has no collision model
    if (!collision_model)
        return;

    // If enabling collision, add to collision system if not already processed
    if (collide && !collision_model->HasImplementation()) {
        if (GetSystem() && GetSystem()->GetCollisionSystem())
            GetSystem()->GetCollisionSystem()->Add(collision_model);
        return;
    }

    // If disabling collision, remove from collision system if already processed
    if (!collide && collision_model->HasImplementation()) {
        if (GetSystem() && GetSystem()->GetCollisionSystem())
            GetSystem()->GetCollisionSystem()->Remove(collision_model);
        return;
    }
}

bool ChBody::GetCollide() const {
    return collide;
}

void ChBody::AddCollisionModelsToSystem(ChCollisionSystem* coll_sys) const {
    if (collide && collision_model)
        coll_sys->Add(collision_model);
}

void ChBody::RemoveCollisionModelsFromSystem(ChCollisionSystem* coll_sys) const {
    if (collision_model)
        coll_sys->Remove(collision_model);
}

void ChBody::SyncCollisionModels() {
    // Sync model only if
    //    (1) a collision model was specified for the body
    //    (2) the body is set to participate in collisions
    // ChCollisionModel::SyncPosition will further check that the collision model was actually processed (through
    // BindAll or BindItem) by the current collision system.

    if (GetCollisionModel() && GetCollide())
        GetCollisionModel()->SyncPosition();
}

// ---------------------------------------------------------------------------

geometry::ChAABB ChBody::GetTotalAABB() {
    if (GetCollisionModel())
        return GetCollisionModel()->GetBoundingBox();

    return geometry::ChAABB();  // default: inverted bounding box
}

void ChBody::ContactableGetStateBlock_x(ChState& x) {
    x.segment(0, 3) = GetCoord().pos.eigen();
    x.segment(3, 4) = GetCoord().rot.eigen();
}

void ChBody::ContactableGetStateBlock_w(ChStateDelta& w) {
    w.segment(0, 3) = GetPos_dt().eigen();
    w.segment(3, 3) = GetWvel_loc().eigen();
}

void ChBody::ContactableIncrementState(const ChState& x, const ChStateDelta& dw, ChState& x_new) {
    IntStateIncrement(0, x_new, x, 0, dw);
}

ChVector<> ChBody::GetContactPoint(const ChVector<>& loc_point, const ChState& state_x) {
    ChCoordsys<> csys(state_x.segment(0, 7));
    return csys.TransformPointLocalToParent(loc_point);
}

ChVector<> ChBody::GetContactPointSpeed(const ChVector<>& loc_point,
                                        const ChState& state_x,
                                        const ChStateDelta& state_w) {
    ChCoordsys<> csys(state_x.segment(0, 7));
    ChVector<> abs_vel(state_w.segment(0, 3));
    ChVector<> loc_omg(state_w.segment(3, 3));
    ChVector<> abs_omg = csys.TransformDirectionLocalToParent(loc_omg);

    return abs_vel + Vcross(abs_omg, loc_point);
}

ChVector<> ChBody::GetContactPointSpeed(const ChVector<>& abs_point) {
    ChVector<> m_p1_loc = Point_World2Body(abs_point);
    return PointSpeedLocalToParent(m_p1_loc);
}

ChCoordsys<> ChBody::GetCsysForCollisionModel() {
    return ChCoordsys<>(GetFrame_REF_to_abs().coord);
}

void ChBody::ContactForceLoadResidual_F(const ChVector<>& F, const ChVector<>& abs_point, ChVectorDynamic<>& R) {
    ChVector<> m_p1_loc = Point_World2Body(abs_point);
    ChVector<> force1_loc = Dir_World2Body(F);
    ChVector<> torque1_loc = Vcross(m_p1_loc, force1_loc);
    R.segment(GetOffset_w() + 0, 3) += F.eigen();
    R.segment(GetOffset_w() + 3, 3) += torque1_loc.eigen();
}

void ChBody::ContactForceLoadQ(const ChVector<>& F,
                               const ChVector<>& point,
                               const ChState& state_x,
                               ChVectorDynamic<>& Q,
                               int offset) {
    ChCoordsys<> csys(state_x.segment(0, 7));
    ChVector<> point_loc = csys.TransformPointParentToLocal(point);
    ChVector<> force_loc = csys.TransformDirectionParentToLocal(F);
    ChVector<> torque_loc = Vcross(point_loc, force_loc);
    Q.segment(offset + 0, 3) = F.eigen();
    Q.segment(offset + 3, 3) = torque_loc.eigen();
}

void ChBody::ComputeJacobianForContactPart(const ChVector<>& abs_point,
                                           ChMatrix33<>& contact_plane,
                                           ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
                                           ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
                                           ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
                                           bool second) {
    /*
    ChVector<> p1 = Point_World2Body(abs_point);
    ChStarMatrix33<> Ps1(p1);

    ChMatrix33<> Jx1 = contact_plane.transpose();
    if (!second)
        Jx1 *= -1;

    ChMatrix33<> Jr1 = contact_plane.transpose() * GetA() * Ps1;
    if (!second)
        Jr1 *= -1;

    jacobian_tuple_N.Get_Cq().segment(0, 3) = Jx1.row(0);
    jacobian_tuple_U.Get_Cq().segment(0, 3) = Jx1.row(1);
    jacobian_tuple_V.Get_Cq().segment(0, 3) = Jx1.row(2);

    jacobian_tuple_N.Get_Cq().segment(3, 3) = Jr1.row(0);
    jacobian_tuple_U.Get_Cq().segment(3, 3) = Jr1.row(1);
    jacobian_tuple_V.Get_Cq().segment(3, 3) = Jr1.row(2);
    */

    // UNROLLED VERSION - FASTER
    ChVector<> p1 = Point_World2Body(abs_point);
    double temp00 =
        Amatrix(0, 2) * contact_plane(0, 0) + Amatrix(1, 2) * contact_plane(1, 0) + Amatrix(2, 2) * contact_plane(2, 0);
    double temp01 =
        Amatrix(0, 2) * contact_plane(0, 1) + Amatrix(1, 2) * contact_plane(1, 1) + Amatrix(2, 2) * contact_plane(2, 1);
    double temp02 =
        Amatrix(0, 2) * contact_plane(0, 2) + Amatrix(1, 2) * contact_plane(1, 2) + Amatrix(2, 2) * contact_plane(2, 2);
    double temp10 =
        Amatrix(0, 1) * contact_plane(0, 0) + Amatrix(1, 1) * contact_plane(1, 0) + Amatrix(2, 1) * contact_plane(2, 0);
    double temp11 =
        Amatrix(0, 1) * contact_plane(0, 1) + Amatrix(1, 1) * contact_plane(1, 1) + Amatrix(2, 1) * contact_plane(2, 1);
    double temp12 =
        Amatrix(0, 1) * contact_plane(0, 2) + Amatrix(1, 1) * contact_plane(1, 2) + Amatrix(2, 1) * contact_plane(2, 2);
    double temp20 =
        Amatrix(0, 0) * contact_plane(0, 0) + Amatrix(1, 0) * contact_plane(1, 0) + Amatrix(2, 0) * contact_plane(2, 0);
    double temp21 =
        Amatrix(0, 0) * contact_plane(0, 1) + Amatrix(1, 0) * contact_plane(1, 1) + Amatrix(2, 0) * contact_plane(2, 1);
    double temp22 =
        Amatrix(0, 0) * contact_plane(0, 2) + Amatrix(1, 0) * contact_plane(1, 2) + Amatrix(2, 0) * contact_plane(2, 2);

    // Jx1 =
    // [ c00, c10, c20]
    // [ c01, c11, c21]
    // [ c02, c12, c22]
    // Jr1 = [ p1y*(temp00) - p1z*(temp10), p1z*(temp20) - p1x*(temp00), p1x*(temp10) - p1y*(temp20);
    //       p1y*(temp01) - p1z*(temp11), p1z*(temp21) - p1x*(temp01), p1x*(temp11) - p1y*(temp21);
    //       p1y*(temp02) - p1z*(temp12), p1z*(temp22) - p1x*(temp02), p1x*(temp12) - p1y*(temp22)];
    if (!second) {
        jacobian_tuple_N.Get_Cq()(0) = -contact_plane(0, 0);
        jacobian_tuple_N.Get_Cq()(1) = -contact_plane(1, 0);
        jacobian_tuple_N.Get_Cq()(2) = -contact_plane(2, 0);
        jacobian_tuple_N.Get_Cq()(3) = -p1.y() * temp00 + p1.z() * temp10;
        jacobian_tuple_N.Get_Cq()(4) = -p1.z() * temp20 + p1.x() * temp00;
        jacobian_tuple_N.Get_Cq()(5) = -p1.x() * temp10 + p1.y() * temp20;

        jacobian_tuple_U.Get_Cq()(0) = -contact_plane(0, 1);
        jacobian_tuple_U.Get_Cq()(1) = -contact_plane(1, 1);
        jacobian_tuple_U.Get_Cq()(2) = -contact_plane(2, 1);
        jacobian_tuple_U.Get_Cq()(3) = -p1.y() * temp01 + p1.z() * temp11;
        jacobian_tuple_U.Get_Cq()(4) = -p1.z() * temp21 + p1.x() * temp01;
        jacobian_tuple_U.Get_Cq()(5) = -p1.x() * temp11 + p1.y() * temp21;

        jacobian_tuple_V.Get_Cq()(0) = -contact_plane(0, 2);
        jacobian_tuple_V.Get_Cq()(1) = -contact_plane(1, 2);
        jacobian_tuple_V.Get_Cq()(2) = -contact_plane(2, 2);
        jacobian_tuple_V.Get_Cq()(3) = -p1.y() * temp02 + p1.z() * temp12;
        jacobian_tuple_V.Get_Cq()(4) = -p1.z() * temp22 + p1.x() * temp02;
        jacobian_tuple_V.Get_Cq()(5) = -p1.x() * temp12 + p1.y() * temp22;
    } else {
        jacobian_tuple_N.Get_Cq()(0) = contact_plane(0, 0);
        jacobian_tuple_N.Get_Cq()(1) = contact_plane(1, 0);
        jacobian_tuple_N.Get_Cq()(2) = contact_plane(2, 0);
        jacobian_tuple_N.Get_Cq()(3) = p1.y() * temp00 - p1.z() * temp10;
        jacobian_tuple_N.Get_Cq()(4) = p1.z() * temp20 - p1.x() * temp00;
        jacobian_tuple_N.Get_Cq()(5) = p1.x() * temp10 - p1.y() * temp20;

        jacobian_tuple_U.Get_Cq()(0) = contact_plane(0, 1);
        jacobian_tuple_U.Get_Cq()(1) = contact_plane(1, 1);
        jacobian_tuple_U.Get_Cq()(2) = contact_plane(2, 1);
        jacobian_tuple_U.Get_Cq()(3) = p1.y() * temp01 - p1.z() * temp11;
        jacobian_tuple_U.Get_Cq()(4) = p1.z() * temp21 - p1.x() * temp01;
        jacobian_tuple_U.Get_Cq()(5) = p1.x() * temp11 - p1.y() * temp21;

        jacobian_tuple_V.Get_Cq()(0) = contact_plane(0, 2);
        jacobian_tuple_V.Get_Cq()(1) = contact_plane(1, 2);
        jacobian_tuple_V.Get_Cq()(2) = contact_plane(2, 2);
        jacobian_tuple_V.Get_Cq()(3) = p1.y() * temp02 - p1.z() * temp12;
        jacobian_tuple_V.Get_Cq()(4) = p1.z() * temp22 - p1.x() * temp02;
        jacobian_tuple_V.Get_Cq()(5) = p1.x() * temp12 - p1.y() * temp22;
    }
}

void ChBody::ComputeJacobianForRollingContactPart(
    const ChVector<>& abs_point,
    ChMatrix33<>& contact_plane,
    ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_N,
    ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_U,
    ChVariableTupleCarrier_1vars<6>::type_constraint_tuple& jacobian_tuple_V,
    bool second) {
    ChMatrix33<> Jr1 = contact_plane.transpose() * GetA();
    if (!second)
        Jr1 *= -1;

    jacobian_tuple_N.Get_Cq().segment(0, 3).setZero();
    jacobian_tuple_U.Get_Cq().segment(0, 3).setZero();
    jacobian_tuple_V.Get_Cq().segment(0, 3).setZero();
    jacobian_tuple_N.Get_Cq().segment(3, 3) = Jr1.row(0);
    jacobian_tuple_U.Get_Cq().segment(3, 3) = Jr1.row(1);
    jacobian_tuple_V.Get_Cq().segment(3, 3) = Jr1.row(2);
}

ChVector<> ChBody::GetAppliedForce() {
    return GetSystem()->GetBodyAppliedForce(this);
}

ChVector<> ChBody::GetAppliedTorque() {
    return GetSystem()->GetBodyAppliedTorque(this);
}

ChVector<> ChBody::GetContactForce() {
    return GetSystem()->GetContactContainer()->GetContactableForce(this);
}

ChVector<> ChBody::GetContactTorque() {
    return GetSystem()->GetContactContainer()->GetContactableTorque(this);
}

// ---------------------------------------------------------------------------

void ChBody::LoadableGetVariables(std::vector<ChVariables*>& mvars) {
    mvars.push_back(&Variables());
}

void ChBody::LoadableStateIncrement(const unsigned int off_x,
                                    ChState& x_new,
                                    const ChState& x,
                                    const unsigned int off_v,
                                    const ChStateDelta& Dv) {
    IntStateIncrement(off_x, x_new, x, off_v, Dv);
}

void ChBody::LoadableGetStateBlock_x(int block_offset, ChState& mD) {
    mD.segment(block_offset + 0, 3) = GetCoord().pos.eigen();
    mD.segment(block_offset + 3, 4) = GetCoord().rot.eigen();
}

void ChBody::LoadableGetStateBlock_w(int block_offset, ChStateDelta& mD) {
    mD.segment(block_offset + 0, 3) = GetPos_dt().eigen();
    mD.segment(block_offset + 3, 3) = GetWvel_loc().eigen();
}

void ChBody::ComputeNF(
    const double U,              // x coordinate of application point in absolute space
    const double V,              // y coordinate of application point in absolute space
    const double W,              // z coordinate of application point in absolute space
    ChVectorDynamic<>& Qi,       // Return result of N'*F  here, maybe with offset block_offset
    double& detJ,                // Return det[J] here
    const ChVectorDynamic<>& F,  // Input F vector, size is 6, it is {Force,Torque} in absolute coords.
    ChVectorDynamic<>* state_x,  // if != 0, update state (pos. part) to this, then evaluate Q
    ChVectorDynamic<>* state_w   // if != 0, update state (speed part) to this, then evaluate Q
) {
    ChVector<> abs_pos(U, V, W);
    ChVector<> absF(F.segment(0, 3));
    ChVector<> absT(F.segment(3, 3));
    ChVector<> body_absF;
    ChVector<> body_locT;
    ChCoordsys<> bodycoord;
    if (state_x)
        bodycoord = state_x->segment(0, 7);  // the numerical jacobian algo might change state_x
    else
        bodycoord = coord;

    // compute Q components F,T, given current state of body 'bodycoord'. Note T in Q is in local csys, F is an abs csys
    body_absF = absF;
    body_locT = bodycoord.rot.RotateBack(absT + ((abs_pos - bodycoord.pos) % absF));
    Qi.segment(0, 3) = body_absF.eigen();
    Qi.segment(3, 3) = body_locT.eigen();
    detJ = 1;  // not needed because not used in quadrature.
}

// ---------------------------------------------------------------------------
// FILE I/O

void ChBody::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChBody>();

    // serialize parent class
    ChPhysicsItem::ArchiveOut(marchive);
    // serialize parent class
    ChBodyFrame::ArchiveOut(marchive);

    // serialize all member data:

    marchive << CHNVP(fixed);
    marchive << CHNVP(collide);

    marchive << CHNVP(bflags);
    bool mflag;  // more readable flag output in case of ASCII in/out
    mflag = BFlagGet(BodyFlag::LIMITSPEED);
    marchive << CHNVP(mflag, "limit_speed");
    mflag = BFlagGet(BodyFlag::NOGYROTORQUE);
    marchive << CHNVP(mflag, "no_gyro_torque");
    mflag = BFlagGet(BodyFlag::USESLEEPING);
    marchive << CHNVP(mflag, "use_sleeping");
    mflag = BFlagGet(BodyFlag::SLEEPING);
    marchive << CHNVP(mflag, "is_sleeping");

    marchive << CHNVP(marklist, "markers");
    marchive << CHNVP(forcelist, "forces");

    marchive << CHNVP(body_id);
    marchive << CHNVP(collision_model);
    marchive << CHNVP(gyro);
    marchive << CHNVP(Xforce);
    marchive << CHNVP(Xtorque);
    // marchive << CHNVP(Force_acc); // not useful in serialization
    // marchive << CHNVP(Torque_acc);// not useful in serialization
    marchive << CHNVP(density);
    marchive << CHNVP(variables);
    marchive << CHNVP(max_speed);
    marchive << CHNVP(max_wvel);
    marchive << CHNVP(sleep_time);
    marchive << CHNVP(sleep_minspeed);
    marchive << CHNVP(sleep_minwvel);
    marchive << CHNVP(sleep_starttime);
}

/// Method to allow de serialization of transient data from archives.
void ChBody::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChBody>();

    // deserialize parent class
    ChPhysicsItem::ArchiveIn(marchive);
    // deserialize parent class
    ChBodyFrame::ArchiveIn(marchive);

    // stream in all member data:

    marchive >> CHNVP(fixed);
    marchive >> CHNVP(collide);

    marchive >> CHNVP(bflags);
    bool mflag;  // more readable flag output in case of ASCII in/out
    if (marchive.in(CHNVP(mflag, "limit_speed")))
        BFlagSet(BodyFlag::LIMITSPEED, mflag);
    if (marchive.in(CHNVP(mflag, "no_gyro_torque")))
        BFlagSet(BodyFlag::NOGYROTORQUE, mflag);
    if (marchive.in(CHNVP(mflag, "use_sleeping")))
        BFlagSet(BodyFlag::USESLEEPING, mflag);
    if (marchive.in(CHNVP(mflag, "is_sleeping")))
        BFlagSet(BodyFlag::SLEEPING, mflag);

    std::vector<std::shared_ptr<ChMarker>> tempmarkers;
    std::vector<std::shared_ptr<ChForce>> tempforces;
    marchive >> CHNVP(tempmarkers, "markers");
    marchive >> CHNVP(tempforces, "forces");
    // trick needed because the "Add...() functions are required
    RemoveAllMarkers();
    for (auto i : tempmarkers) {
        AddMarker(i);
    }
    RemoveAllForces();
    for (auto i : tempforces) {
        AddForce(i);
    }

    marchive >> CHNVP(body_id);

    std::shared_ptr<ChCollisionModel> collision_model_temp;  ///< pointer to the collision model
    marchive >> CHNVP(collision_model_temp, "collision_model");

    marchive >> CHNVP(gyro);
    marchive >> CHNVP(Xforce);
    marchive >> CHNVP(Xtorque);
    // marchive << CHNVP(Force_acc); // not useful in serialization
    // marchive << CHNVP(Torque_acc);// not useful in serialization
    marchive >> CHNVP(density);
    marchive >> CHNVP(variables);
    marchive >> CHNVP(max_speed);
    marchive >> CHNVP(max_wvel);
    marchive >> CHNVP(sleep_time);
    marchive >> CHNVP(sleep_minspeed);
    marchive >> CHNVP(sleep_minwvel);
    marchive >> CHNVP(sleep_starttime);

    // INITIALIZATION-BY-METHODS
    if (marchive.CanTolerateMissingTokens()) {
        bool temp_tolerate_missing_tokens = marchive.GetTolerateMissingTokens();
        marchive.TryTolerateMissingTokens(true);

        bool _c_SetBodyFixed;
        if (marchive.in(CHNVP(_c_SetBodyFixed)))
            SetBodyFixed(_c_SetBodyFixed);

        double _c_SetMass;
        if (marchive.in(CHNVP(_c_SetMass)))
            SetMass(_c_SetMass);

        ChVector<> _c_SetInertiaXX;
        if (marchive.in(CHNVP(_c_SetInertiaXX)))
            SetInertiaXX(_c_SetInertiaXX);

        ChVector<> _c_SetInertiaXY;
        if (marchive.in(CHNVP(_c_SetInertiaXY)))
            SetInertiaXY(_c_SetInertiaXY);

        marchive.TryTolerateMissingTokens(temp_tolerate_missing_tokens);
    }
}

void ChBody::StreamOutstate(ChStreamOutBinary& mstream) {
    // Do not serialize parent classes and do not
    // implement versioning, because this must be efficient
    // and will be used just for domain decomposition.
    mstream << coord.pos.x();
    mstream << coord.pos.y();
    mstream << coord.pos.z();
    mstream << coord.rot.e0();
    mstream << coord.rot.e1();
    mstream << coord.rot.e2();
    mstream << coord.rot.e3();
    mstream << coord_dt.pos.x();
    mstream << coord_dt.pos.y();
    mstream << coord_dt.pos.z();
    mstream << coord_dt.rot.e0();
    mstream << coord_dt.rot.e1();
    mstream << coord_dt.rot.e2();
    mstream << coord_dt.rot.e3();
}

void ChBody::StreamInstate(ChStreamInBinary& mstream) {
    // Do not serialize parent classes and do not
    // implement versioning, because this must be efficient
    // and will be used just for domain decomposition.
    mstream >> coord.pos.x();
    mstream >> coord.pos.y();
    mstream >> coord.pos.z();
    mstream >> coord.rot.e0();
    mstream >> coord.rot.e1();
    mstream >> coord.rot.e2();
    mstream >> coord.rot.e3();
    SetCoord(coord);
    mstream >> coord_dt.pos.x();
    mstream >> coord_dt.pos.y();
    mstream >> coord_dt.pos.z();
    mstream >> coord_dt.rot.e0();
    mstream >> coord_dt.rot.e1();
    mstream >> coord_dt.rot.e2();
    mstream >> coord_dt.rot.e3();
    SetCoord_dt(coord_dt);

    Update();
    SyncCollisionModels();
}

}  // end namespace chrono
