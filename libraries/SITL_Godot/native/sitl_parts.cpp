// Copyright (C) 2026 David Buzz
// SPDX-License-Identifier: GPL-3.0-or-later
// Part of ArduPilot and released under its licence: see COPYING.txt.
#include "sitl_parts.h" // mirrors scripts/ap_*.gd: change both

#include <algorithm>

namespace SITLParts {

static const double PI = 3.14159265358979323846;

static double clampd(double v, double lo, double hi) { return std::min(std::max(v, lo), hi); }
static double signd(double v) { return v > 0 ? 1.0 : (v < 0 ? -1.0 : 0.0); }

Basis Basis::orthonormalized() const // Gram-Schmidt, as Godot's
{
    Basis b;
    b.x = x.normalized();
    b.y = (y - b.x * b.x.dot(y)).normalized();
    b.z = (z - b.x * b.x.dot(z) - b.y * b.y.dot(z)).normalized();
    return b;
}

double servo_fraction(const int32_t *pwm, int channels, int servo, double pwm_min, double pwm_max)
{
    if (servo < 1 || servo > channels || pwm_max <= pwm_min) {
        return -1.0;
    }
    const double v = pwm[servo - 1];
    if (v < 500.0) { // disabled channel outputs 0
        return -1.0;
    }
    return clampd((v - pwm_min) / (pwm_max - pwm_min), 0.0, 1.0);
}

Vec3 Rotor::propwash_at(const Vec3 &point, const Vec3 &v_air, const Vec3 &span_dir, double span) const
{
    if (slipstream <= 0.0) {
        return Vec3();
    }
    const Vec3 axis = world.basis.y.normalized();
    const Vec3 rel = point - world.origin;
    const double behind = -rel.dot(axis);
    if (behind <= 0.0) {
        return Vec3();
    }
    const double r = 0.5 * diameter / std::sqrt(2.0); // slipstream has half disc area
    const Vec3 offset = rel + axis * behind;
    double covered = 0.0;
    const Vec3 u = span_dir - axis * span_dir.dot(axis);
    const double a = u.length_squared();
    if (span <= 0.0 || a < 1e-6) {
        covered = offset.length() <= r ? 1.0 : 0.0;
    } else {
        const double b = 2.0 * offset.dot(u);
        const double disc = b * b - 4.0 * a * (offset.length_squared() - r * r);
        if (disc <= 0.0) {
            return Vec3();
        }
        const double s1 = std::max((-b - std::sqrt(disc)) / (2.0 * a), -0.5 * span);
        const double s2 = std::min((-b + std::sqrt(disc)) / (2.0 * a), 0.5 * span);
        covered = std::max(s2 - s1, 0.0) / span; // span fraction inside slipstream
    }
    if (covered <= 0.0) {
        return Vec3();
    }
    const double v = std::max(v_air.dot(axis), 0.0);
    const double w = slipstream;
    return axis * (std::sqrt((1.0 - covered) * v * v + covered * (v + w) * (v + w)) - v);
}

double Surface::effective_aspect_ratio() const
{
    if (aspect_ratio > 0.0) {
        return aspect_ratio;
    }
    return chord > 0.0 ? span / chord : 1.0;
}

void Surface::coefficients(double a, double d, double &cl, double &cd) const
{
    const double ar = effective_aspect_ratio();
    const double slope = 2.0 * PI * ar / (2.0 + std::sqrt(ar * ar + 4.0)); // finite wing (Helmbold)
    const double theta = std::acos(2.0 * flap_chord_fraction - 1.0);
    const double a_eff = a + (1.0 - (theta - std::sin(theta)) / PI) * d; // thin aerofoil theory
    const double a0 = stall_angle * PI / 180.0;
    const double m = 50.0;
    const double e1 = std::exp(-m * (a_eff - a0));
    const double e2 = std::exp(m * (a_eff + a0));
    const double sigma = (1.0 + e1 + e2) / ((1.0 + e1) * (1.0 + e2)); // Beard & McLain 2012, 4.2
    const double cl_linear = cl0 + slope * a_eff;
    const double s = std::sin(a_eff);
    const double c = std::cos(a_eff);
    const double cl_plate = 2.0 * signd(a_eff) * s * s * c;
    cl = (1.0 - sigma) * cl_linear + sigma * cl_plate;
    const double cd_induced = cl_linear * cl_linear / (PI * oswald * ar);
    cd = cd0 + (1.0 - sigma) * cd_induced + sigma * 2.0 * s * s;
    const double sd = std::sin(d);
    cd += 0.9 * std::pow(flap_chord_fraction, 1.38) * sd * sd;
}

int Parts::add_hinge(const Hinge &h)
{
    hinges.push_back(h);
    return int(hinges.size()) - 1;
}

int Parts::add_joint_hinge(const JointHinge &j)
{
    joint_hinges.push_back(j);
    joint_hinges.back().held = j.built_angle;
    return int(joint_hinges.size()) - 1;
}

int Parts::add_rotor(const Rotor &r)
{
    rotors.push_back(r);
    order.push_back({ROTOR, int(rotors.size()) - 1});
    return int(rotors.size()) - 1;
}

int Parts::add_surface(const Surface &s)
{
    surfaces.push_back(s);
    order.push_back({SURFACE, int(surfaces.size()) - 1});
    return int(surfaces.size()) - 1;
}

int Parts::add_wheel(const Wheel &w)
{
    wheels.push_back(w);
    order.push_back({WHEEL, int(wheels.size()) - 1});
    return int(wheels.size()) - 1;
}

int Parts::add_floater(const Floater &f)
{
    floaters.push_back(f);
    order.push_back({FLOATER, int(floaters.size()) - 1});
    return int(floaters.size()) - 1;
}

int Parts::add_heli_rotor(const HeliRotor &r)
{
    heli_rotors.push_back(r);
    order.push_back({HELI_ROTOR, int(heli_rotors.size()) - 1});
    return int(heli_rotors.size()) - 1;
}

void Parts::step_heli_rotor(HeliRotor &r, const Body &body, const int32_t *pwm, int channels, double dt, Wrench &out)
{
    const double rsc = std::max(servo_fraction(pwm, channels, r.rsc_servo, r.pwm_min, r.pwm_max), 0.0);
    const double target = r.nominal_rpm * clampd(rsc / 0.66, 0.0, 1.0); // full speed at 66% throttle
    const double slew = r.nominal_rpm * dt / std::max(r.spool_time, 0.01);
    r.rpm = clampd(target, r.rpm - slew, r.rpm + slew);
    const double rpm_frac = r.nominal_rpm > 0.0 ? r.rpm / r.nominal_rpm : 0.0;
    const double s1 = servo_fraction(pwm, channels, r.swash_servo[0], r.pwm_min, r.pwm_max);
    const double s2 = servo_fraction(pwm, channels, r.swash_servo[1], r.pwm_min, r.pwm_max);
    const double s3 = servo_fraction(pwm, channels, r.swash_servo[2], r.pwm_min, r.pwm_max);
    if (s1 < 0.0 || s2 < 0.0 || s3 < 0.0 || rpm_frac < 0.05) {
        r.thrust = r.tpp_x = r.tpp_y = 0.0;
        return;
    }
    double collective = r.collective_min + (r.collective_max - r.collective_min) * (s1 + s2 + s3) / 3.0;
    const double roll_cyclic = r.cyclic_scale * PI / 180.0 * (s1 - s2);
    const double pitch_cyclic = r.cyclic_scale * PI / 180.0 * 1.15 * ((s1 + s2) * 0.5 - s3);
    const Transform xform = place(body, r.hinge, r.local);
    const Basis b = xform.basis.orthonormalized();
    const Vec3 w_local = b.xform_inv(body.angular);
    const double tau = std::max(r.flap_time_constant, 0.005);
    r.tpp_x = clampd(r.tpp_x + ((roll_cyclic - r.tpp_x) / tau - w_local.z) * dt, -0.3, 0.3); // flapping lags the body rate
    r.tpp_y = clampd(r.tpp_y + ((pitch_cyclic - r.tpp_y) / tau + w_local.x) * dt, -0.3, 0.3);
    const double v_axial = (body.velocity_at(xform.origin) - body.wind).dot(b.y);
    const double v_blade = 0.75 * (r.rpm * 2.0 * PI / 60.0) * (0.5 * r.diameter);
    if (v_blade > 1.0) {
        collective -= std::atan(v_axial / v_blade) * 180.0 / PI; // climb inflow reduces effective pitch
    }
    r.thrust = r.thrust_scale * collective * rpm_frac * rpm_frac;
    const Vec3 dir_local = Vec3{-r.tpp_x, 1.0, -r.tpp_y}.normalized();
    out.add(b.xform(dir_local * r.thrust), xform.origin - (body.xform.origin + body.com));
    const double reaction = r.torque_ratio * (std::fabs(r.thrust) + 1.5 * r.thrust_scale * rpm_frac * rpm_frac);
    out.torque += b.xform(Vec3{-r.hub_stiffness * r.tpp_y, r.spin * reaction, r.hub_stiffness * r.tpp_x});
}

void Parts::reset()
{
    for (Rotor &r : rotors) {
        r.static_thrust = r.thrust = 0.0;
    }
    for (Wheel &w : wheels) {
        w.compression = 0.0;
    }
    for (HeliRotor &r : heli_rotors) {
        r.rpm = r.thrust = r.tpp_x = r.tpp_y = 0.0;
    }
    for (JointHinge &j : joint_hinges) {
        j.turned = j.last_raw = 0.0;
        j.held = j.built_angle;
    }
}

Transform Parts::place(const Body &body, int hinge, const Transform &local) const
{
    return hinge >= 0 ? body.xform * hinges[hinge].local * local : body.xform * local;
}

static const double JOINT_GAIN = 30.0; // (deg/s) per degree of error

void Parts::step_joint_hinge(JointHinge &j, const int32_t *pwm, int channels, World *world)
{
    if (world == nullptr) {
        return;
    }
    const Basis a = world->body_basis(j.body_a).orthonormalized();
    const Basis b = world->body_basis(j.body_b).orthonormalized();
    const Vec3 va = a.xform(j.ref_a);
    const Vec3 vb = b.xform(j.ref_b);
    const double raw = std::atan2(b.xform(j.axis_b).dot(vb.cross(va)), vb.dot(va));
    double moved = raw - j.last_raw;
    moved -= 2.0 * PI * std::floor((moved + PI) / (2.0 * PI)); // counts past half a turn
    j.turned += moved;
    j.last_raw = raw;
    j.angle = j.turned * 180.0 / PI + j.built_angle;
    const double f = servo_fraction(pwm, channels, j.servo, j.pwm_min, j.pwm_max);
    const double target = f >= 0.0 ? j.angle_min + (j.angle_max - j.angle_min) * f : j.held;
    j.held = target;
    double rate = clampd(JOINT_GAIN * (target - j.angle), -j.max_rate, j.max_rate);
    if (j.continuous) {
        const double way = j.angle_max >= j.angle_min ? 1.0 : -1.0;
        rate = f >= 0.0 ? way * (2.0 * f - 1.0) * j.max_rate : 0.0;
    }
    world->drive_joint(j.joint, rate * PI / 180.0, j.max_torque);
}

void Parts::step_hinge(Hinge &h, const Body &body, const int32_t *pwm, int channels, double dt)
{
    const double f = servo_fraction(pwm, channels, h.servo, h.pwm_min, h.pwm_max);
    double target = f >= 0.0 ? h.angle_min + (h.angle_max - h.angle_min) * f : h.held;
    h.held = target;
    if (h.sheet) { // boom swings to leeward
        const Basis above = h.parent >= 0 ? body.xform.basis * hinges[h.parent].local.basis : body.xform.basis;
        const Basis b = (above * h.rest.basis).orthonormalized(); // X mast, Y across boom
        const double side = (body.wind - body.linear).dot(b.y);
        if (std::fabs(side) > 0.15) {
            h.lee = side > 0.0 ? 1.0 : -1.0;
        }
        target = std::fabs(target) * h.lee;
    }
    if (h.max_rate > 0.0) {
        const double step = h.max_rate * dt;
        target = clampd(target, h.angle - step, h.angle + step);
    }
    h.angle = target;
    const double a = h.angle * PI / 180.0;
    Transform turn; // about X
    turn.basis.y = {0, std::cos(a), std::sin(a)};
    turn.basis.z = {0, -std::sin(a), std::cos(a)};
    h.local = h.rest * turn;
    if (h.parent >= 0) {
        h.local = hinges[h.parent].local * h.local;
    }
}

void Parts::step_wheel(Wheel &w, const Body &body, const int32_t *pwm, int channels, double dt, Wrench &out, World *world)
{
    const Transform xform = place(body, w.hinge, w.local);
    const Vec3 up = xform.basis.y.normalized();
    const Vec3 top = xform.origin + up * w.travel;
    const double length = w.travel + w.radius;
    Vec3 contact, normal;
    const double last = w.compression;
    if (world == nullptr || !world->ray(top, top - up * length, contact, normal)) {
        w.compression = 0.0;
        return;
    }
    w.compression = length - (top - contact).length();
    const double rate = clampd(dt > 0.0 ? (w.compression - last) / dt : 0.0, -10.0, 10.0); // clamp: teleport would spike damping
    const double load = std::max(w.spring * w.compression + w.damping * rate, 0.0);
    const Vec3 v = body.velocity_at(contact);
    Vec3 fwd = xform.basis.z;
    fwd = (fwd - normal * fwd.dot(normal)).normalized();
    const Vec3 side = normal.cross(fwd);
    const double v_fwd = v.dot(fwd);
    const double v_side = v.dot(side);
    Vec3 force = normal * load; // along normal, not the strut
    force += side * (-w.grip * load * clampd(v_side / 0.2, -1.0, 1.0)); // smoothed near zero for stability
    force += fwd * (-w.rolling_resistance * load * clampd(v_fwd / 0.2, -1.0, 1.0));
    if (w.drive_servo > 0 && w.max_drive_force > 0.0) {
        const double f = servo_fraction(pwm, channels, w.drive_servo, w.drive_pwm_min, w.drive_pwm_max);
        if (f >= 0.0) {
            double demand = 2.0 * f - 1.0;
            if (w.free_speed > 0.0) {
                demand -= v_fwd / w.free_speed;
            }
            force += fwd * clampd(demand * w.max_drive_force, -w.grip * load, w.grip * load);
        }
    }
    out.add(force, contact - (body.xform.origin + body.com));
}

void Parts::step_floater(Floater &f, const Body &body, Wrench &out)
{
    const Transform xform = place(body, f.hinge, f.local);
    double density = body.air_density;
    double submerged = 1.0;
    if (!f.air) {
        if (!body.water_enabled) {
            return;
        }
        density = body.water_density;
        submerged = clampd((body.water_level - xform.origin.y) / std::max(f.height, 0.001) + 0.5, 0.0, 1.0);
        if (submerged <= 0.0) {
            return;
        }
    }
    Vec3 force{0, density * 9.80665 * f.volume * submerged, 0};
    if (f.drag_area.length_squared() > 0.0 || f.damping > 0.0) {
        const Basis b = xform.basis.orthonormalized();
        const Vec3 v = b.xform_inv(body.velocity_at(xform.origin));
        const Vec3 local{-0.5 * density * f.drag_area.x * v.x * std::fabs(v.x) - f.damping * v.x,
            -0.5 * density * f.drag_area.y * v.y * std::fabs(v.y) - f.damping * v.y,
            -0.5 * density * f.drag_area.z * v.z * std::fabs(v.z) - f.damping * v.z};
        force += b.xform(local * submerged);
    }
    out.add(force, xform.origin - (body.xform.origin + body.com));
}

void Parts::step_rotor(Rotor &r, const Body &body, const int32_t *pwm, int channels, double dt, Wrench &out)
{
    const double f = servo_fraction(pwm, channels, r.servo, r.pwm_min, r.pwm_max);
    double t = std::max(f, 0.0);
    double forward = 1.0;
    if (r.bidirectional && f >= 0.0) {
        forward = signd(2.0 * f - 1.0);
        t = std::fabs(2.0 * f - 1.0);
    }
    const double demand = forward * r.max_thrust * ((1.0 - r.thrust_expo) * t + r.thrust_expo * t * t);
    if (r.time_constant > 0.0) {
        r.static_thrust += (demand - r.static_thrust) * clampd(dt / r.time_constant, 0.0, 1.0);
    } else {
        r.static_thrust = demand;
    }
    const Vec3 axis = r.world.basis.y.normalized();
    const Vec3 v_air = body.velocity_at(r.world.origin) - body.wind;
    const double inflow = v_air.dot(axis);
    r.thrust = r.static_thrust;
    if (r.pitch_speed > 0.0 && r.max_thrust > 0.0 && r.static_thrust > 0.0) {
        const double v_pitch = r.pitch_speed * std::sqrt(r.static_thrust / r.max_thrust); // pitch speed scales with rpm
        if (v_pitch > 0.01) {
            r.thrust *= clampd(1.0 - inflow / v_pitch, 0.0, 1.5);
        }
    }
    const double area = PI * 0.25 * r.diameter * r.diameter;
    if (r.thrust > 0.0 && area > 0.0) {
        const double v_axial = std::max(inflow, 0.0);
        const double v_induced = -0.5 * v_axial + std::sqrt(0.25 * v_axial * v_axial + r.thrust / (2.0 * body.air_density * area));
        r.slipstream = 2.0 * v_induced; // induced velocity doubles far behind
    } else {
        r.slipstream = 0.0;
    }
    Vec3 force = axis * r.thrust;
    if (r.inplane_drag > 0.0) {
        force += (v_air - axis * inflow) * (-r.inplane_drag * std::fabs(r.thrust));
    }
    out.add(force, r.world.origin - (body.xform.origin + body.com));
    out.torque += axis * (-r.spin * r.torque_ratio * r.thrust); // ccw reaction torque is clockwise
    if (r.max_thrust > 0.0) {
        r.turned += r.spin * 200.0 * std::sqrt(std::fabs(r.static_thrust) / r.max_thrust) * dt;
    }
}

void Parts::step_surface(Surface &s, const Body &body, const int32_t *pwm, int channels, Wrench &out)
{
    if (s.flap_servo > 0) {
        const double f = servo_fraction(pwm, channels, s.flap_servo, s.flap_pwm_min, s.flap_pwm_max);
        if (f >= 0.0) {
            s.flap = (s.flap_angle_min + (s.flap_angle_max - s.flap_angle_min) * f) * PI / 180.0;
        }
    }
    const Transform xform = place(body, s.hinge, s.local);
    const Basis b = xform.basis.orthonormalized();
    Vec3 v_air;
    double rho = body.air_density;
    if (s.water) { // wind does not move water
        const double reach = std::max(std::max(std::fabs(b.x.y) * s.span, std::fabs(b.z.y) * s.chord), 0.02);
        double wet = 0.0;
        if (body.water_enabled) {
            wet = clampd((body.water_level - xform.origin.y) / reach + 0.5, 0.0, 1.0);
        }
        if (wet <= 0.0) {
            return;
        }
        rho = body.water_density * wet; // only the wetted part
        v_air = body.velocity_at(xform.origin);
    } else {
        v_air = body.velocity_at(xform.origin) - body.wind;
        Vec3 wash;
        for (const Rotor &r : rotors) {
            wash += r.propwash_at(xform.origin, v_air, b.x, s.span);
        }
        v_air += wash;
    }
    Vec3 v_local = b.xform_inv(v_air);
    v_local.x = 0.0; // strip theory: ignore spanwise flow
    const double speed = v_local.length();
    if (speed < 0.1) {
        return;
    }
    double alpha = std::atan2(-v_local.y, v_local.z);
    if (s.downwash != 0.0) {
        alpha -= s.downwash * clampd(body.alpha, -0.3, 0.3); // downwash collapses at wing stall
    }
    double cl, cd;
    s.coefficients(alpha, s.flap, cl, cd);
    const double q = 0.5 * rho * speed * speed;
    const double area = s.span * s.chord;
    const Vec3 wind_dir = v_local * (-1.0 / speed);
    const Vec3 lift_dir = Vec3{1, 0, 0}.cross(wind_dir);
    const Vec3 force = lift_dir * (q * area * cl) + wind_dir * (q * area * cd);
    out.add(b.xform(force), xform.origin - (body.xform.origin + body.com));
    if (s.cm0 != 0.0) {
        out.torque += b.xform(Vec3{-q * area * s.chord * s.cm0, 0.0, 0.0}); // nose-up moment is about -X
    }
}

void Parts::step(Body &body, const int32_t *pwm, int channels, double dt, Wrench &out, World *world)
{
    const Basis b = body.xform.basis.orthonormalized();
    const Vec3 v_body = b.xform_inv(body.linear - body.wind);
    if (own_alpha) {
        body.alpha = std::fabs(v_body.z) + std::fabs(v_body.y) > 0.1 ? std::atan2(-v_body.y, v_body.z) : 0.0;
    }
    for (Hinge &h : hinges) { // first: children need the pose
        step_hinge(h, body, pwm, channels, dt);
    }
    for (JointHinge &j : joint_hinges) {
        step_joint_hinge(j, pwm, channels, world);
    }
    for (Rotor &r : rotors) { // where each is now
        r.world = place(body, r.hinge, r.local);
    }
    for (const Slot &s : order) {
        switch (s.kind) {
        case ROTOR:
            step_rotor(rotors[s.index], body, pwm, channels, dt, out);
            break;
        case SURFACE:
            step_surface(surfaces[s.index], body, pwm, channels, out);
            break;
        case WHEEL:
            step_wheel(wheels[s.index], body, pwm, channels, dt, out, world);
            break;
        case FLOATER:
            step_floater(floaters[s.index], body, out);
            break;
        case HELI_ROTOR:
            step_heli_rotor(heli_rotors[s.index], body, pwm, channels, dt, out);
            break;
        }
    }
    if (drag_area.length_squared() > 0.0) {
        const Vec3 local{-0.5 * body.air_density * drag_area.x * v_body.x * std::fabs(v_body.x),
            -0.5 * body.air_density * drag_area.y * v_body.y * std::fabs(v_body.y),
            -0.5 * body.air_density * drag_area.z * v_body.z * std::fabs(v_body.z)};
        out.force += b.xform(local);
    }
}

} // namespace SITLParts
