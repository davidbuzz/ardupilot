// Copyright (C) 2026 David Buzz
// SPDX-License-Identifier: GPL-3.0-or-later
// Part of ArduPilot and released under its licence: see COPYING.txt.
#include "jolt_sim.h" // bake reader; Godot's Jolt settings
#include "sitl_parts.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <cfenv>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace JPH;
using SITLParts::Transform;

namespace {

const ObjectLayer STATIC = 0, MOVING = 1;
const float MARGIN_FRACTION = 0.08f; // Godot's collision_margin_fraction
const double GRAVITY = 9.80665;
const double LAKE_LEVEL = -0.4, LAKE_DEPTH = 2.2; // main.gd's

struct BroadPhaseLayers final : BroadPhaseLayerInterface {
    uint GetNumBroadPhaseLayers() const override { return 2; }
    BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer layer) const override { return BroadPhaseLayer(uint8(layer)); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char *GetBroadPhaseLayerName(BroadPhaseLayer layer) const override { return "layer"; }
#endif
};
struct ObjectVsBroadPhase final : ObjectVsBroadPhaseLayerFilter {
    bool ShouldCollide(ObjectLayer a, BroadPhaseLayer b) const override { return a == MOVING || b == BroadPhaseLayer(uint8(MOVING)); }
};
struct ObjectPairs final : ObjectLayerPairFilter {
    bool ShouldCollide(ObjectLayer a, ObjectLayer b) const override { return a == MOVING || b == MOVING; }
};

SITLParts::Vec3 vec(Vec3Arg v) { return {v.GetX(), v.GetY(), v.GetZ()}; }
Vec3 vec(const SITLParts::Vec3 &v) { return Vec3(float(v.x), float(v.y), float(v.z)); }
SITLParts::Vec3 to_ned(const SITLParts::Vec3 &v) { return {v.z, -v.x, -v.y}; } // glTF: +Z forward, +Y up

struct Pose {
    Vec3 position = Vec3::sZero();
    Quat rotation = Quat::sIdentity();
    Transform transform() const
    {
        const Mat44 m = Mat44::sRotation(rotation);
        return {{vec(m.GetAxisX()), vec(m.GetAxisY()), vec(m.GetAxisZ())}, vec(position)};
    }
};

std::istream &operator>>(std::istream &in, Pose &p)
{
    float v[7];
    for (float &f : v) {
        in >> f;
    }
    p.position = Vec3(v[0], v[1], v[2]);
    p.rotation = Quat(v[3], v[4], v[5], v[6]).Normalized();
    return in;
}

bool jolt_started = false;

struct QuietFloats { // SITL traps faults Jolt raises
    fenv_t saved;
    QuietFloats() { feholdexcept(&saved); }
    ~QuietFloats() { fesetenv(&saved); }
};

bool numbers(std::istream &in, double *v, int count) // bake writes reals only
{
    for (int i = 0; i < count; i++) {
        in >> v[i];
    }
    return !in.fail();
}

} // namespace

struct JoltSim final : SITLParts::World {
    BroadPhaseLayers broad_phase_layers;
    ObjectVsBroadPhase object_vs_broad_phase;
    ObjectPairs object_pairs;
    PhysicsSystem physics;
    TempAllocatorImpl temp{32 * 1024 * 1024};
    JobSystemSingleThreaded jobs{cMaxPhysicsJobs}; // one body: threads only cost
    std::vector<BodyID> bodies; // the first is the vehicle
    std::vector<Pose> starts;
    std::vector<HingeConstraint *> joints;
    SITLParts::Parts parts;
    SITLParts::Body body;
    SITLParts::Vec3 last_velocity;
    bool have_last_velocity = false;
    double launch_accel = 0, launch_length = 0; // rail push on servo 7
    SITLParts::Vec3 launch_dir, launch_from;
    bool held = false; // held aloft until servo 7
    double mass = 1; // the vehicle body's (kg)
    SITLParts::Vec3 release_velocity;

    bool load(const char *path, double lake_depth, std::string &error);

    SITLParts::Basis body_basis(int index) override
    {
        return Pose{Vec3::sZero(), physics.GetBodyInterfaceNoLock().GetRotation(bodies[index])}.transform().basis;
    }

    void drive_joint(int joint, double rad_per_s, double max_torque) override // as Godot's hinge motor
    {
        HingeConstraint *c = joints[joint];
        c->SetMotorState(EMotorState::Velocity);
        c->SetTargetAngularVelocity(float(-rad_per_s));
        c->GetMotorSettings().mMinTorqueLimit = float(-max_torque);
        c->GetMotorSettings().mMaxTorqueLimit = float(max_torque);
    }

    bool ray(const SITLParts::Vec3 &from, const SITLParts::Vec3 &to, SITLParts::Vec3 &hit, SITLParts::Vec3 &normal) override
    {
        const RRayCast cast(RVec3(vec(from)), vec(to - from));
        RayCastResult result;
        if (!physics.GetNarrowPhaseQuery().CastRay(cast, result, {}, {}, IgnoreSingleBodyFilter(bodies[0]))) {
            return false;
        }
        const RVec3 point = cast.GetPointOnRay(result.mFraction);
        BodyLockRead lock(physics.GetBodyLockInterfaceNoLock(), result.mBodyID);
        if (!lock.Succeeded()) {
            return false;
        }
        hit = vec(Vec3(point));
        normal = vec(lock.GetBody().GetWorldSpaceSurfaceNormal(result.mSubShapeID2, point));
        return true;
    }
};

bool JoltSim::load(const char *path, double lake_depth, std::string &error)
{
    std::ifstream file(path);
    if (!file) {
        error = std::string("cannot read ") + path;
        return false;
    }
    physics.Init(1024, 0, 8192, 4096, broad_phase_layers, object_vs_broad_phase, object_pairs);
    PhysicsSettings settings; // as Godot sets them
    settings.mBaumgarte = 0.2f;
    settings.mSpeculativeContactDistance = 0.02f;
    settings.mPenetrationSlop = 0.02f;
    settings.mLinearCastThreshold = 0.75f;
    settings.mLinearCastMaxPenetration = 0.25f;
    settings.mNumVelocitySteps = 10;
    settings.mNumPositionSteps = 2;
    settings.mMinVelocityForRestitution = 1.0f;
    settings.mAllowSleeping = false;
    physics.SetPhysicsSettings(settings);
    physics.SetGravity(Vec3(0, -float(GRAVITY), 0));
    physics.SetCombineFriction([](const Body &a, const SubShapeID &, const Body &b, const SubShapeID &) {
        return std::abs(std::min(a.GetFriction(), b.GetFriction()));
    });
    physics.SetCombineRestitution([](const Body &a, const SubShapeID &, const Body &b, const SubShapeID &) {
        return std::clamp(a.GetRestitution() + b.GetRestitution(), 0.0f, 1.0f);
    });
    BodyInterface &bi = physics.GetBodyInterfaceNoLock();

    struct PendingJoint {
        int a = 0, b = 0;
        Pose pose;
    };
    std::vector<PendingJoint> pending_joints;
    struct Pending { // body awaiting its shapes
        std::string name;
        float mass = 1, friction = 1, bounce = 0;
        Vec3 inertia = Vec3::sReplicate(1), com = Vec3::sZero();
        Pose pose;
        StaticCompoundShapeSettings shapes;
        int count = 0;
    };
    std::vector<Pending> pending;
    std::string line, kind;
    int version = 0;
    while (std::getline(file, line)) {
        std::istringstream in(line);
        if (!(in >> kind)) {
            continue;
        }
        if (kind == "sitlbake") {
            in >> version;
        } else if (kind == "model") {
            continue;
        } else if (kind == "unsupported") {
            error = std::string(path) + " cannot be flown here yet:" + line.substr(kind.size());
            return false;
        } else if (kind == "env") {
            double v[4];
            numbers(in, v, 4);
            body.air_density = v[0];
            body.water_enabled = v[1] != 0.0;
            body.water_level = v[2];
            body.water_density = v[3];
        } else if (kind == "drag") {
            in >> parts.drag_area.x >> parts.drag_area.y >> parts.drag_area.z;
        } else if (kind == "body") {
            Pending p;
            float v[7];
            in >> p.name >> p.mass;
            for (int i = 0; i < 6; i++) {
                in >> v[i];
            }
            p.inertia = Vec3(v[0], v[1], v[2]);
            p.com = Vec3(v[3], v[4], v[5]);
            in >> p.pose >> p.friction >> p.bounce;
            pending.push_back(p);
        } else if (kind == "shape") {
            size_t index = 0;
            std::string shape;
            float a = 0, b = 0, c = 0, margin = 0;
            Pose pose;
            in >> index >> shape;
            if (index >= pending.size()) {
                error = "shape before its body";
                return false;
            }
            ShapeRefC made;
            if (shape == "box") {
                in >> a >> b >> c >> margin >> pose;
                made = new BoxShape(Vec3(a, b, c), std::min(margin, std::min({a, b, c}) * MARGIN_FRACTION));
            } else if (shape == "sphere") {
                in >> a >> pose;
                made = new SphereShape(a);
            } else if (shape == "capsule") {
                in >> a >> b >> pose; // radius, whole height
                made = new CapsuleShape(std::max(0.5f * b - a, 0.0001f), a);
            } else if (shape == "cylinder") {
                in >> a >> b >> margin >> pose; // radius, height
                made = new CylinderShape(0.5f * b, a, std::min(margin, std::min(0.5f * b, a) * MARGIN_FRACTION));
            } else {
                error = "shape of unknown kind " + shape;
                return false;
            }
            pending[index].shapes.AddShape(pose.position, pose.rotation, made);
            pending[index].count++;
        } else if (kind == "hinge") {
            SITLParts::Hinge h;
            Pose pose;
            double v[7];
            in >> h.parent >> pose;
            numbers(in, v, 7);
            h.servo = int(v[0]);
            h.pwm_min = v[1];
            h.pwm_max = v[2];
            h.angle_min = v[3];
            h.angle_max = v[4];
            h.max_rate = v[5];
            h.sheet = v[6] != 0.0;
            h.rest = pose.transform();
            parts.add_hinge(h);
        } else if (kind == "joint") {
            PendingJoint j;
            in >> j.a >> j.b >> j.pose;
            pending_joints.push_back(j);
        } else if (kind == "jointhinge") {
            SITLParts::JointHinge j;
            double v[9];
            in >> j.joint;
            numbers(in, v, 9);
            j.servo = int(v[0]);
            j.pwm_min = v[1];
            j.pwm_max = v[2];
            j.angle_min = v[3];
            j.angle_max = v[4];
            j.max_rate = v[5];
            j.max_torque = v[6];
            j.continuous = v[7] != 0.0;
            j.built_angle = v[8];
            parts.add_joint_hinge(j);
        } else if (kind == "rotor") {
            SITLParts::Rotor r;
            Pose pose;
            double v[12]; // order: ArduPilotRotor.native_params
            in >> r.hinge >> pose;
            numbers(in, v, 12);
            r.servo = int(v[0]);
            r.pwm_min = v[1];
            r.pwm_max = v[2];
            r.max_thrust = v[3];
            r.thrust_expo = v[4];
            r.bidirectional = v[5] != 0.0;
            r.torque_ratio = v[6];
            r.spin = v[7];
            r.time_constant = v[8];
            r.diameter = v[9];
            r.pitch_speed = v[10];
            r.inplane_drag = v[11];
            r.local = pose.transform();
            parts.add_rotor(r);
        } else if (kind == "surface") {
            SITLParts::Surface s;
            Pose pose;
            double v[16]; // order: ArduPilotSurface.native_params
            in >> s.hinge >> pose;
            numbers(in, v, 16);
            s.water = v[0] != 0.0;
            s.span = v[1];
            s.chord = v[2];
            s.aspect_ratio = v[3];
            s.cl0 = v[4];
            s.cd0 = v[5];
            s.cm0 = v[6];
            s.oswald = v[7];
            s.stall_angle = v[8];
            s.downwash = v[9];
            s.flap_servo = int(v[10]);
            s.flap_pwm_min = v[11];
            s.flap_pwm_max = v[12];
            s.flap_angle_min = v[13];
            s.flap_angle_max = v[14];
            s.flap_chord_fraction = v[15];
            s.local = pose.transform();
            parts.add_surface(s);
        } else if (kind == "wheel") {
            SITLParts::Wheel w;
            Pose pose;
            double v[11];
            in >> w.hinge >> pose;
            numbers(in, v, 11);
            w.radius = v[0];
            w.travel = v[1];
            w.spring = v[2];
            w.damping = v[3];
            w.rolling_resistance = v[4];
            w.grip = v[5];
            w.drive_servo = int(v[6]);
            w.drive_pwm_min = v[7];
            w.drive_pwm_max = v[8];
            w.max_drive_force = v[9];
            w.free_speed = v[10];
            w.local = pose.transform();
            parts.add_wheel(w);
        } else if (kind == "helirotor") {
            SITLParts::HeliRotor r;
            Pose pose;
            double v[17];
            in >> r.hinge >> pose;
            numbers(in, v, 17);
            r.swash_servo[0] = int(v[0]);
            r.swash_servo[1] = int(v[1]);
            r.swash_servo[2] = int(v[2]);
            r.rsc_servo = int(v[3]);
            r.pwm_min = v[4];
            r.pwm_max = v[5];
            r.collective_min = v[6];
            r.collective_max = v[7];
            r.cyclic_scale = v[8];
            r.nominal_rpm = v[9];
            r.spool_time = v[10];
            r.thrust_scale = v[11];
            r.torque_ratio = v[12];
            r.hub_stiffness = v[13];
            r.flap_time_constant = v[14];
            r.spin = v[15];
            r.diameter = v[16];
            r.local = pose.transform();
            parts.add_heli_rotor(r);
        } else if (kind == "floater") {
            SITLParts::Floater f;
            Pose pose;
            double v[7];
            in >> f.hinge >> pose;
            numbers(in, v, 7);
            f.air = v[0] != 0.0;
            f.volume = v[1];
            f.height = v[2];
            f.damping = v[3];
            f.drag_area = {v[4], v[5], v[6]};
            f.local = pose.transform();
            parts.add_floater(f);
        } else {
            error = "bake record of unknown kind " + kind;
            return false;
        }
        if (in.fail()) {
            error = "short bake record: " + line;
            return false;
        }
    }
    if (version != 1 || pending.empty()) {
        error = std::string(path) + " is not a version 1 bake with a body in it";
        return false;
    }
    if (body.water_enabled) { // floaters sit in main.gd's lake
        body.water_level = LAKE_LEVEL;
    }
    const float floor = body.water_enabled ? -float(lake_depth > 0.0 ? lake_depth : LAKE_DEPTH) : 0.0f; // no shore: the bed everywhere
    BodyCreationSettings ground(new BoxShape(Vec3(20000, 0.5f, 20000)), RVec3(0, floor - 0.5f, 0), Quat::sIdentity(), EMotionType::Static, STATIC);
    ground.mFriction = 0.8f; // main.gd's ground
    bi.CreateAndAddBody(ground, EActivation::DontActivate);

    Ref<GroupFilterTable> jointed = new GroupFilterTable(uint(pending.size())); // jointed bodies do not collide
    for (const PendingJoint &j : pending_joints) {
        if (j.a < 0 || j.b < 0 || size_t(j.a) >= pending.size() || size_t(j.b) >= pending.size() || j.a == j.b) {
            error = "joint between bodies which are not there";
            return false;
        }
        jointed->DisableCollision(CollisionGroup::SubGroupID(j.a), CollisionGroup::SubGroupID(j.b));
    }
    mass = pending[0].mass;
    for (Pending &p : pending) {
        ShapeRefC shape = p.count > 0 ? p.shapes.Create().Get() : ShapeRefC(new EmptyShape());
        const Vec3 offset = p.com - shape->GetCenterOfMass();
        if (offset.LengthSq() > 1e-12f) { // centre of mass as modelled
            shape = OffsetCenterOfMassShapeSettings(offset, shape).Create().Get();
        }
        BodyCreationSettings s(shape, RVec3(p.pose.position), p.pose.rotation, EMotionType::Dynamic, MOVING);
        s.mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;
        s.mMassPropertiesOverride.mMass = p.mass;
        s.mMassPropertiesOverride.mInertia = Mat44::sScale(p.inertia);
        s.mFriction = p.friction;
        s.mRestitution = p.bounce;
        s.mLinearDamping = 0;
        s.mAngularDamping = 0;
        s.mAllowSleeping = false;
        s.mCollisionGroup = CollisionGroup(jointed, 0, CollisionGroup::SubGroupID(bodies.size()));
        bodies.push_back(bi.CreateAndAddBody(s, EActivation::Activate));
        starts.push_back(p.pose);
    }
    for (const PendingJoint &j : pending_joints) { // modules/jolt_physics/joints/jolt_hinge_joint_3d.cpp
        const Mat44 frame = Mat44::sRotation(j.pose.rotation);
        HingeConstraintSettings hs;
        hs.mSpace = EConstraintSpace::WorldSpace;
        hs.mPoint1 = hs.mPoint2 = RVec3(j.pose.position);
        hs.mHingeAxis1 = hs.mHingeAxis2 = frame.GetAxisZ();
        hs.mNormalAxis1 = hs.mNormalAxis2 = frame.GetAxisX();
        hs.mLimitsMin = -JPH_PI;
        hs.mLimitsMax = JPH_PI;
        Body *a = physics.GetBodyLockInterfaceNoLock().TryGetBody(bodies[j.a]);
        Body *b = physics.GetBodyLockInterfaceNoLock().TryGetBody(bodies[j.b]);
        HingeConstraint *made = static_cast<HingeConstraint *>(hs.Create(*a, *b));
        physics.AddConstraint(made);
        joints.push_back(made);
    }
    for (SITLParts::JointHinge &j : parts.joint_hinges) { // scripts/ap_hinge.gd _bind_joint
        if (j.joint < 0 || size_t(j.joint) >= pending_joints.size()) {
            error = "servo on a joint which is not there";
            return false;
        }
        const PendingJoint &pj = pending_joints[j.joint];
        const SITLParts::Basis frame = pj.pose.transform().basis;
        j.body_a = pj.a;
        j.body_b = pj.b;
        j.ref_a = starts[pj.a].transform().basis.xform_inv(frame.x);
        j.ref_b = starts[pj.b].transform().basis.xform_inv(frame.x);
        j.axis_b = starts[pj.b].transform().basis.xform_inv(frame.z);
    }
    parts.own_alpha = true;
    physics.OptimizeBroadPhase();
    return true;
}

JoltSim *jolt_sim_create(const char *bake_path, double lake_depth, char *error, int error_size)
{
    const QuietFloats quiet;
    if (!jolt_started) {
        RegisterDefaultAllocator();
        Factory::sInstance = new Factory();
        RegisterTypes();
        jolt_started = true;
    }
    JoltSim *sim = new JoltSim();
    std::string why;
    if (!sim->load(bake_path, lake_depth, why)) {
        snprintf(error, size_t(error_size), "%s", why.c_str());
        delete sim;
        return nullptr;
    }
    return sim;
}

void jolt_sim_destroy(JoltSim *sim)
{
    delete sim;
}

void jolt_sim_reset(JoltSim *sim)
{
    const QuietFloats quiet;
    BodyInterface &bi = sim->physics.GetBodyInterfaceNoLock();
    for (size_t i = 0; i < sim->bodies.size(); i++) {
        bi.SetPositionRotationAndVelocity(sim->bodies[i], RVec3(sim->starts[i].position), sim->starts[i].rotation, Vec3::sZero(), Vec3::sZero());
    }
    sim->parts.reset();
    sim->have_last_velocity = false;
}

void jolt_sim_set_heading(JoltSim *sim, double degrees)
{
    const QuietFloats quiet;
    const Quat turn = Quat::sRotation(Vec3::sAxisY(), -DegreesToRadians(float(degrees))); // +Y is up: clockwise is negative
    for (Pose &p : sim->starts) {
        p.position = turn * p.position;
        p.rotation = (turn * p.rotation).Normalized();
    }
    jolt_sim_reset(sim);
}

void jolt_sim_set_launcher(JoltSim *sim, const char *kind, double a, double b, double c) // scripts/main.gd _build_launcher
{
    const QuietFloats quiet;
    BodyInterface &bi = sim->physics.GetBodyInterfaceNoLock();
    const std::string k(kind);
    const SITLParts::Basis facing = sim->starts[0].transform().basis;
    if (k == "drop" || k == "toss") {
        for (Pose &p : sim->starts) {
            p.position += Vec3(0, float(a), 0);
        }
        jolt_sim_reset(sim);
        sim->held = true;
        sim->release_velocity = k == "toss" ? SITLParts::Vec3{0, b, 0} : facing.z * b;
        bi.SetMotionType(sim->bodies[0], EMotionType::Kinematic, EActivation::Activate);
        return;
    }
    if ((k != "sled" && k != "ramp") || a <= 0.0) {
        return;
    }
    const double angle = c * JPH_PI / 180.0;
    sim->launch_accel = a;
    sim->launch_length = b;
    sim->launch_from = vec(sim->starts[0].position);
    sim->launch_dir = (facing.z * std::cos(angle) + SITLParts::Vec3{0, std::sin(angle), 0}).normalized();
    const Vec3 along = vec(sim->launch_dir); // the rail the wheels run up
    const Vec3 across = Vec3::sAxisY().Cross(along).Normalized();
    const Quat turn = Mat44(Vec4(across, 0), Vec4(along.Cross(across), 0), Vec4(along, 0), Vec4(0, 0, 0, 1)).GetQuaternion();
    const Vec3 mid = vec(sim->launch_from) + along * float(0.5 * b) - Vec3(0, 0.30f, 0);
    BodyCreationSettings rail(new BoxShape(Vec3(0.45f, 0.05f, float(0.5 * b))), RVec3(mid), turn, EMotionType::Static, STATIC);
    bi.CreateAndAddBody(rail, EActivation::DontActivate);
}

void jolt_sim_set_wind(JoltSim *sim, double north, double east, double down)
{
    sim->body.wind = {-east, -down, north};
}

static void read_vehicle(JoltSim *sim) // body state for the parts
{
    BodyInterface &bi = sim->physics.GetBodyInterfaceNoLock();
    RVec3 position;
    Quat rotation;
    bi.GetPositionAndRotation(sim->bodies[0], position, rotation);
    sim->body.xform = Pose{Vec3(position), rotation}.transform();
    sim->body.com = vec(Vec3(bi.GetCenterOfMassPosition(sim->bodies[0]) - position));
    sim->body.linear = vec(bi.GetLinearVelocity(sim->bodies[0]));
    sim->body.angular = vec(bi.GetAngularVelocity(sim->bodies[0]));
}

void jolt_sim_step(JoltSim *sim, const uint16_t *pwm, int channels, double dt, JoltSimState *state)
{
    const QuietFloats quiet;
    BodyInterface &bi = sim->physics.GetBodyInterfaceNoLock();
    int32_t outputs[32] = {};
    channels = std::min(channels, 32);
    for (int i = 0; i < channels; i++) {
        outputs[i] = pwm[i];
    }
    read_vehicle(sim);
    SITLParts::Wrench wrench;
    sim->parts.step(sim->body, outputs, channels, dt, wrench, sim);
    const bool fired = channels >= 7 && outputs[6] > 1700; // servo 7, as SITL's launcher
    if (sim->held && fired) {
        sim->held = false;
        bi.SetMotionType(sim->bodies[0], EMotionType::Dynamic, EActivation::Activate);
        bi.SetLinearVelocity(sim->bodies[0], vec(sim->release_velocity));
    } else if (sim->launch_accel > 0.0 && fired && (sim->body.xform.origin - sim->launch_from).length() < sim->launch_length) {
        wrench.force += sim->launch_dir * (sim->mass * sim->launch_accel);
    }
    if (!sim->held) {
        bi.AddForce(sim->bodies[0], vec(wrench.force));
        bi.AddTorque(sim->bodies[0], vec(wrench.torque));
    }
    sim->physics.Update(float(dt), 1, &sim->temp, &sim->jobs);

    read_vehicle(sim); // scripts/ap_vehicle.gd sitl_state, in C++
    const SITLParts::Body &b = sim->body;
    const SITLParts::Basis basis = b.xform.basis.orthonormalized();
    const SITLParts::Vec3 v = b.linear + b.angular.cross(-b.com); // at the origin: the IMU
    SITLParts::Vec3 accel;
    if (sim->have_last_velocity && dt > 0.0) {
        accel = (v - sim->last_velocity) * (1.0 / dt);
    }
    sim->last_velocity = v;
    sim->have_last_velocity = true;
    const SITLParts::Vec3 specific_force = accel - SITLParts::Vec3{0, -GRAVITY, 0};
    const SITLParts::Vec3 gyro = to_ned(basis.xform_inv(b.angular));
    const SITLParts::Vec3 accel_body = to_ned(basis.xform_inv(specific_force));
    const SITLParts::Vec3 position = to_ned(b.xform.origin);
    const SITLParts::Vec3 velocity = to_ned(v);
    const SITLParts::Vec3 columns[3] = {to_ned(basis.z), to_ned(-basis.x), to_ned(-basis.y)}; // TO_NED * basis * TO_NED^T
    const SITLParts::Vec3 *out[4] = {&gyro, &accel_body, &position, &velocity};
    double *into[4] = {state->gyro, state->accel_body, state->position, state->velocity};
    for (int i = 0; i < 4; i++) {
        into[i][0] = out[i]->x;
        into[i][1] = out[i]->y;
        into[i][2] = out[i]->z;
    }
    for (int c = 0; c < 3; c++) {
        state->dcm[0 + c] = columns[c].x;
        state->dcm[3 + c] = columns[c].y;
        state->dcm[6 + c] = columns[c].z;
    }
    state->has_airspeed = !sim->parts.surfaces.empty();
    state->airspeed = std::max(basis.xform_inv(v - b.wind).z, 0.0); // pitot along the nose
    const SITLParts::Vec3 air = basis.xform_inv(b.wind - v); // apparent wind, body frame
    state->has_vane = b.wind.length_squared() > 0.0;
    state->vane_direction = std::remainder(std::atan2(-air.x, air.z) + JPH_PI, 2.0 * JPH_PI); // where it comes from
    state->vane_speed = std::hypot(air.x, air.z);
}

int jolt_sim_poses(JoltSim *sim, double *out, int max_bodies)
{
    BodyInterface &bi = sim->physics.GetBodyInterfaceNoLock();
    const int count = std::min(int(sim->bodies.size()), max_bodies);
    for (int i = 0; i < count; i++) {
        RVec3 position;
        Quat rotation;
        bi.GetPositionAndRotation(sim->bodies[i], position, rotation);
        const double pose[7] = {position.GetX(), position.GetY(), position.GetZ(), rotation.GetX(), rotation.GetY(), rotation.GetZ(), rotation.GetW()};
        memcpy(out + 7 * i, pose, sizeof(pose));
    }
    return count;
}
