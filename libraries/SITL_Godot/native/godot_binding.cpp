// Copyright (C) 2026 David Buzz
// SPDX-License-Identifier: GPL-3.0-or-later
// Part of ArduPilot and released under its licence: see COPYING.txt.
#include "sitl_parts.h" // Godot's view: types converted only

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/transform3d.hpp>

using namespace godot;

static SITLParts::Vec3 vec(const Vector3 &v) { return {v.x, v.y, v.z}; }
static Vector3 vec(const SITLParts::Vec3 &v) { return Vector3(v.x, v.y, v.z); }
static SITLParts::Transform xform(const Transform3D &t)
{
    return {{vec(t.basis.get_column(0)), vec(t.basis.get_column(1)), vec(t.basis.get_column(2))}, vec(t.origin)};
}

class ArduPilotNative : public RefCounted {
    GDCLASS(ArduPilotNative, RefCounted)
    SITLParts::Parts parts;
    SITLParts::Wrench wrench;

protected:
    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("add_rotor", "local", "params"), &ArduPilotNative::add_rotor);
        ClassDB::bind_method(D_METHOD("add_surface", "local", "params"), &ArduPilotNative::add_surface);
        ClassDB::bind_method(D_METHOD("step", "body", "com", "linear", "angular", "wind", "env", "pwm", "dt"), &ArduPilotNative::step);
        ClassDB::bind_method(D_METHOD("get_force"), &ArduPilotNative::get_force);
        ClassDB::bind_method(D_METHOD("get_torque"), &ArduPilotNative::get_torque);
        ClassDB::bind_method(D_METHOD("rotor_turned", "index"), &ArduPilotNative::rotor_turned);
        ClassDB::bind_method(D_METHOD("surface_flap", "index"), &ArduPilotNative::surface_flap);
        ClassDB::bind_method(D_METHOD("reset"), &ArduPilotNative::reset);
    }

public:
    int add_rotor(const Transform3D &local, const PackedFloat64Array &p) // order: ArduPilotRotor.native_params
    {
        ERR_FAIL_COND_V(p.size() != 12, -1);
        SITLParts::Rotor r;
        r.local = xform(local);
        r.servo = int(p[0]);
        r.pwm_min = p[1];
        r.pwm_max = p[2];
        r.max_thrust = p[3];
        r.thrust_expo = p[4];
        r.bidirectional = p[5] != 0.0;
        r.torque_ratio = p[6];
        r.spin = p[7];
        r.time_constant = p[8];
        r.diameter = p[9];
        r.pitch_speed = p[10];
        r.inplane_drag = p[11];
        return parts.add_rotor(r);
    }

    int add_surface(const Transform3D &local, const PackedFloat64Array &p) // order: ArduPilotSurface.native_params
    {
        ERR_FAIL_COND_V(p.size() != 16, -1);
        SITLParts::Surface s;
        s.local = xform(local);
        s.water = p[0] != 0.0;
        s.span = p[1];
        s.chord = p[2];
        s.aspect_ratio = p[3];
        s.cl0 = p[4];
        s.cd0 = p[5];
        s.cm0 = p[6];
        s.oswald = p[7];
        s.stall_angle = p[8];
        s.downwash = p[9];
        s.flap_servo = int(p[10]);
        s.flap_pwm_min = p[11];
        s.flap_pwm_max = p[12];
        s.flap_angle_min = p[13];
        s.flap_angle_max = p[14];
        s.flap_chord_fraction = p[15];
        return parts.add_surface(s);
    }

    void step(const Transform3D &body, const Vector3 &com, const Vector3 &linear, const Vector3 &angular,
            const Vector3 &wind, const PackedFloat64Array &env, const PackedInt32Array &pwm, double dt)
    {
        ERR_FAIL_COND(env.size() != 5);
        SITLParts::Body b;
        b.xform = xform(body);
        b.com = vec(com);
        b.linear = vec(linear);
        b.angular = vec(angular);
        b.wind = vec(wind);
        b.air_density = env[0];
        b.water_enabled = env[1] != 0.0;
        b.water_level = env[2];
        b.water_density = env[3];
        b.alpha = env[4];
        wrench = SITLParts::Wrench();
        parts.step(b, pwm.ptr(), int(pwm.size()), dt, wrench);
    }

    Vector3 get_force() const { return vec(wrench.force); }
    Vector3 get_torque() const { return vec(wrench.torque); }

    double rotor_turned(int i) // since last asked (rad)
    {
        ERR_FAIL_INDEX_V(i, int(parts.rotors.size()), 0.0);
        const double a = parts.rotors[i].turned;
        parts.rotors[i].turned = 0.0;
        return a;
    }

    double surface_flap(int i) const
    {
        ERR_FAIL_INDEX_V(i, int(parts.surfaces.size()), 0.0);
        return parts.surfaces[i].flap;
    }

    void reset() { parts.reset(); }
};

static void initialize(ModuleInitializationLevel level)
{
    if (level == MODULE_INITIALIZATION_LEVEL_SCENE) {
        GDREGISTER_CLASS(ArduPilotNative);
    }
}

static void uninitialize(ModuleInitializationLevel) {}

extern "C" GDExtensionBool GDE_EXPORT ardupilot_native_init(GDExtensionInterfaceGetProcAddress get_proc_address,
        GDExtensionClassLibraryPtr library, GDExtensionInitialization *initialization)
{
    GDExtensionBinding::InitObject init(get_proc_address, library, initialization);
    init.register_initializer(initialize);
    init.register_terminator(uninitialize);
    init.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init.init();
}
