// Copyright (C) 2026 David Buzz
// SPDX-License-Identifier: GPL-3.0-or-later
// Part of ArduPilot and released under its licence: see COPYING.txt.
#pragma once // part force models, engine-free
#include <cmath>
#include <cstdint>
#include <vector>

namespace SITLParts {

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 &operator+=(const Vec3 &o) { x += o.x; y += o.y; z += o.z; return *this; }
    double dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3 &o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
    double length_squared() const { return dot(*this); }
    double length() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const { const double l = length(); return l > 0 ? *this * (1.0 / l) : Vec3(); }
};

struct Basis { // columns: the axes
    Vec3 x{1, 0, 0}, y{0, 1, 0}, z{0, 0, 1};
    Vec3 xform(const Vec3 &v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 xform_inv(const Vec3 &v) const { return {x.dot(v), y.dot(v), z.dot(v)}; } // orthonormal only
    Basis operator*(const Basis &o) const { return {xform(o.x), xform(o.y), xform(o.z)}; }
    Basis orthonormalized() const;
};

struct Transform {
    Basis basis;
    Vec3 origin;
    Transform operator*(const Transform &o) const { return {basis * o.basis, basis.xform(o.origin) + origin}; }
};

struct Hinge { // turns its children about X
    Transform rest; // from its parent, at zero angle
    int parent = -1; // another hinge, or the body
    int servo = 1;
    double pwm_min = 1000, pwm_max = 2000;
    double angle_min = -30, angle_max = 30, max_rate = 300; // deg, deg/s
    bool sheet = false; // sail sheet: sign follows airflow
    double angle = 0, held = 0, lee = 1; // state (deg; lee side +1/-1)
    Transform local; // from the body, now
};

struct JointHinge { // servo on a body joint
    int joint = 0, body_a = 0, body_b = 0;
    int servo = 1;
    double pwm_min = 1000, pwm_max = 2000;
    double angle_min = -30, angle_max = 30, max_rate = 300, max_torque = 2;
    bool continuous = false; // servo sets rate, not angle
    double built_angle = 0; // joint angle as built (deg)
    Vec3 ref_a, ref_b, axis_b; // set where it is built
    double turned = 0, last_raw = 0, held = 0, angle = 0; // state
};

struct Rotor { // propeller: thrust along local +Y
    Transform local; // from body or hinge
    int hinge = -1;
    int servo = 1;
    double pwm_min = 1000, pwm_max = 2000;
    double max_thrust = 10, thrust_expo = 0.65;
    bool bidirectional = false;
    double torque_ratio = 0.016, spin = 1; // spin: ccw is +1
    double time_constant = 0.03, diameter = 0.25;
    double pitch_speed = 0, inplane_drag = 0;
    double static_thrust = 0, thrust = 0, slipstream = 0; // state
    double turned = 0; // for the picture (rad)
    Transform world;
    Vec3 propwash_at(const Vec3 &point, const Vec3 &v_air, const Vec3 &span_dir, double span) const;
};

struct Surface { // lift +Y, origin quarter chord
    Transform local;
    int hinge = -1;
    bool water = false;
    double span = 1, chord = 0.2, aspect_ratio = 0;
    double cl0 = 0, cd0 = 0.015, cm0 = 0, oswald = 0.85;
    double stall_angle = 15, downwash = 0;
    int flap_servo = 0;
    double flap_pwm_min = 1000, flap_pwm_max = 2000;
    double flap_angle_min = -20, flap_angle_max = 20, flap_chord_fraction = 0.25;
    double flap = 0; // state (rad)
    double effective_aspect_ratio() const;
    void coefficients(double a, double d, double &cl, double &cd) const;
};

struct Wheel { // strut on Y, rolls along Z
    Transform local;
    int hinge = -1;
    double radius = 0.05, travel = 0.05, spring = 500, damping = 20;
    double rolling_resistance = 0.02, grip = 0.9;
    int drive_servo = 0;
    double drive_pwm_min = 1000, drive_pwm_max = 2000, max_drive_force = 0, free_speed = 0;
    double compression = 0; // state (m)
};

struct Floater { // buoyancy: hull or envelope
    Transform local;
    int hinge = -1;
    bool air = false;
    double volume = 0.001, height = 0.1, damping = 0;
    Vec3 drag_area;
};

struct HeliRotor { // main rotor, H3-120 swash
    Transform local;
    int hinge = -1;
    int swash_servo[3] = {1, 2, 3}, rsc_servo = 8;
    double pwm_min = 1000, pwm_max = 2000;
    double collective_min = -25, collective_max = 25, cyclic_scale = 10; // blade pitch (deg)
    double nominal_rpm = 1500, spool_time = 5, thrust_scale = 9;
    double torque_ratio = 0.035, hub_stiffness = 15, flap_time_constant = 0.04;
    double spin = 1; // cw seen from above is +1
    double diameter = 1.4;
    double rpm = 0, thrust = 0, tpp_x = 0, tpp_y = 0; // state; tpp right, aft (rad)
};

struct World { // what a part may ask
    virtual ~World() {}
    virtual bool ray(const Vec3 &from, const Vec3 &to, Vec3 &hit, Vec3 &normal) = 0; // the vehicle itself excluded
    virtual Basis body_basis(int body) = 0;
    virtual void drive_joint(int joint, double rad_per_s, double max_torque) = 0;
};

struct Body { // the rigid body they hang on
    Transform xform;
    Vec3 com; // from the origin, world axes
    Vec3 linear, angular, wind;
    double air_density = 1.225;
    bool water_enabled = false;
    double water_level = 0, water_density = 1000;
    double alpha = 0; // whole vehicle's (rad)
    Vec3 velocity_at(const Vec3 &point) const { return linear + angular.cross(point - (xform.origin + com)); }
};

struct Wrench { // about the centre of mass
    Vec3 force, torque;
    void add(const Vec3 &f, const Vec3 &arm) { force += f; torque += arm.cross(f); }
};

class Parts {
public:
    int add_hinge(const Hinge &h); // before what hangs on it
    int add_joint_hinge(const JointHinge &j);
    int add_rotor(const Rotor &r);
    int add_surface(const Surface &s);
    int add_wheel(const Wheel &w);
    int add_floater(const Floater &f);
    int add_heli_rotor(const HeliRotor &r);
    void step(Body &body, const int32_t *pwm, int channels, double dt, Wrench &out, World *world = nullptr); // hinges, then in the order added
    void reset();
    std::vector<Hinge> hinges;
    std::vector<JointHinge> joint_hinges;
    std::vector<Rotor> rotors;
    std::vector<Surface> surfaces;
    std::vector<Wheel> wheels;
    std::vector<Floater> floaters;
    std::vector<HeliRotor> heli_rotors;
    Vec3 drag_area; // body CdA per axis (m^2)
    bool own_alpha = false; // work out body.alpha here

private:
    enum Kind { ROTOR, SURFACE, WHEEL, FLOATER, HELI_ROTOR };
    struct Slot {
        Kind kind;
        int index;
    };
    std::vector<Slot> order;
    Transform place(const Body &body, int hinge, const Transform &local) const;
    void step_hinge(Hinge &h, const Body &body, const int32_t *pwm, int channels, double dt);
    void step_joint_hinge(JointHinge &j, const int32_t *pwm, int channels, World *world);
    void step_rotor(Rotor &r, const Body &body, const int32_t *pwm, int channels, double dt, Wrench &out);
    void step_surface(Surface &s, const Body &body, const int32_t *pwm, int channels, Wrench &out);
    void step_wheel(Wheel &w, const Body &body, const int32_t *pwm, int channels, double dt, Wrench &out, World *world);
    void step_floater(Floater &f, const Body &body, Wrench &out);
    void step_heli_rotor(HeliRotor &r, const Body &body, const int32_t *pwm, int channels, double dt, Wrench &out);
};

double servo_fraction(const int32_t *pwm, int channels, int servo, double pwm_min, double pwm_max); // 0..1, -1 without output

} // namespace SITLParts
