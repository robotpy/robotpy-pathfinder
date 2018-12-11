#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

// Use this to release the gil
typedef py::call_guard<py::gil_scoped_release> release_gil;

#include "grpl/pf.h"
namespace pf = grpl::pf;


//
// Module declaration
//

#define REPR(x) " " #x "=" + std::to_string(__inst.x)


PYBIND11_MODULE(_pathfinder, m) {

    m.attr("__version__") = VERSION_INFO;

    //
    // TODO: figure out which eigen args/return value should never be copied, and
    // change them to use Eigen::Ref<const MatrixType>??
    // 

    // pf/constants.h

    m.attr("POSITION") = pf::POSITION;
    m.attr("VELOCITY") = pf::VELOCITY;
    m.attr("ACCELERATION") = pf::ACCELERATION;
    m.attr("JERK") = pf::JERK;

    auto c = m.def_submodule("constants");
    c.attr("PI") = pf::constants::PI;
    c.attr("epsilon") = pf::constants::epsilon;
    c.attr("almost_inf") = pf::constants::almost_inf;
    c.attr("default_acceptable_error") = pf::constants::default_acceptable_error;
    c.attr("profile_kinematics_order") = pf::constants::profile_kinematics_order;
    c.attr("profile_limits_order") = pf::constants::profile_limits_order;

    //
    // pf/coupled
    //

    auto cp = m.def_submodule("coupled");
    cp.doc() =  "Coupled / tank / differential drivetrain models.\n"
                "\n"
                "The grpl::pf::coupled namespace contains all models and other structures for use\n"
                "with coupled-style drivetrains.\n"
                "\n"
                "A coupled-style drivetrain defines a drivetrain with individually controlled, parallel\n"
                "left and right sides, also known as a tank drive or differential drive. May also refer\n"
                "to skid-steer systems.\n";

    // causal_trajectory_generator.h
    // auto cp_ctg = py::class_<grpl::pf::coupled::causal_trajectory_generator>(m, "causal_trajectory_generator")
    //     .def("generate", &pf::coupled::causal_trajectory_generator::generate,
    //          py::arg("chassis"), py::arg("curve_begin"), py::arg("curve_end"), py::arg("profile"), py::arg("last"), py::arg("time"));

    // pf/coupled/chassis.h
    auto cp_chassis = py::class_<pf::coupled::chassis>(cp, "chassis")
        .def(py::init<pf::coupled::chassis::transmission_t &,pf::coupled::chassis::transmission_t &,double,double,double>(),
             py::arg("transmission_left"), py::arg("transmission_right"), py::arg("wheel_radius"), py::arg("track_radius"), py::arg("mass"))
        .def("mass", &pf::coupled::chassis::mass)
        .def("track_radius", &pf::coupled::chassis::track_radius)
        .def("wheel_radius", &pf::coupled::chassis::wheel_radius)
        .def("transmission_left", &pf::coupled::chassis::transmission_left)
        .def("transmission_right", &pf::coupled::chassis::transmission_right)
        .def("linear_vel_limit", &pf::coupled::chassis::linear_vel_limit,
             py::arg("config"), py::arg("curvature"))
        .def("acceleration_limits", &pf::coupled::chassis::acceleration_limits,
             py::arg("config"), py::arg("curvature"), py::arg("velocity"))
        .def("split", &pf::coupled::chassis::split,
             py::arg("centre"));

    cp_chassis.doc() =
        "Mathematical model representation of a coupled (tank / differential) drivetrain.\n"
        "\n"
        "Chassis contains members for the transmissions (motors), as well as other configurations\n"
        "regarding the chassis (track radius, wheel radius, mass, etc).\n"
        "\n"
        "The chassis mirrors the physical \"layout\" of the drivetrain.\n"
        "\n"
        "@ref grpl::pf::coupled::causal_trajectory_generator\n"
        "";

    // pf/coupled/state.h
    auto cp_state = py::class_<pf::coupled::state>(cp, "state")
        .def(py::init<>())
        .def_readwrite("time", &pf::coupled::state::time)
        .def_readwrite("curvature", &pf::coupled::state::curvature)
        .def_readwrite("dcurvature", &pf::coupled::state::dcurvature)
        .def_readwrite("config", &pf::coupled::state::config)
        .def_readwrite("kinematics", &pf::coupled::state::kinematics)
        .def_readwrite("finished", &pf::coupled::state::finished);
        // TODO: __repr__
    
    cp_state.doc() =
        "The state of a coupled drivetrain at any point in time, as a single state within a\n"
        "trajectory.\n";

    auto cp_wheel_state = py::class_<pf::coupled::wheel_state>(cp, "wheel_state")
        .def(py::init<>())
        .def_readwrite("time", &pf::coupled::wheel_state::time)
        .def_readwrite("position", &pf::coupled::wheel_state::position)
        .def_readwrite("kinematics", &pf::coupled::wheel_state::kinematics)
        .def_readwrite("voltage", &pf::coupled::wheel_state::voltage)
        .def_readwrite("current", &pf::coupled::wheel_state::current)
        .def_readwrite("finished", &pf::coupled::wheel_state::finished);
        // TODO: __repr__
    
    cp_wheel_state.doc() =
        "The state of a wheel (side) of the coupled drivetrain at any point in time, primarily\n"
        "for use with encoders / other following regimes.\n";

    //
    // pf/path
    //

    auto path = m.def_submodule("path");

    // pf/path/arc_parametrizer.h
    auto path_arc_parametrizer = py::class_<pf::path::arc_parameterizer>(path, "arc_parameterizer")
        .def(py::init<>())
        .def("configure", &pf::path::arc_parameterizer::configure,
             py::arg("max_arc_length"), py::arg("max_delta_curvature"))
        .def("curve_count", &pf::path::arc_parameterizer::curve_count, release_gil(),
             py::arg("spline"), py::arg("t_lo") = 0, py::arg("t_hi") = 1, py::arg("count") = 0)
        .def("curve_count", release_gil(), []( &__inst, std::vector<spline<2>> splines) {
            return __inst.curve_count(splines.begin(), splines.end());
        }, py::arg("splines"))
        .def("parameterize", &pf::path::arc_parameterizer::parameterize, release_gil(),
             py::arg("spline"), py::arg("curve_begin"), py::arg("max_curve_count"), py::arg("t_lo"), py::arg("t_hi"))
        .def("parameterize", &pf::path::arc_parameterizer::parameterize, release_gil(),
             py::arg("spline_begin"), py::arg("spline_end"), py::arg("curve_begin"), py::arg("max_curve_count"))
        .def("has_overrun", &pf::path::arc_parameterizer::has_overrun)

    // pf/path/arc.h

    auto path_arc2d = py::class_<pf::path::arc2d, pf::path::curve<2>>(path, "arc2d")
        .def(py::init<>())
        .def(py::init<pf::path::arc2d::vector_t, pf::path::arc2d::vector_t, pf::path::arc2d::vector_t>(),
             py::arg("start"), py::arg("mid"), py::arg("end"))
        .def("position", &pf::path::arc2d::position,
             py::arg("s"))
        .def("derivative", &pf::path::arc2d::derivative,
             py::arg("s"))
        .def("curvature", &pf::path::arc2d::curvature,
             py::arg("s"))
        .def("dcurvature", &pf::path::arc2d::dcurvature,
             py::arg("s"))
        .def("length", &pf::path::arc2d::length);
    
    path_arc2d.doc() = 
        "A 2-Dimensional Circular Arc\n"
        "\n"
        "Implementation of a 2-Dimensional circular arc, parameterized to arc length 's'.\n";
    
    // pf/path/augmented_arc2d.h

    auto path_augmented_arc2d = py::class_<pf::path::augmented_arc2d, pf::path::arc2d>(path, "augmented_arc2d")
        .def(py::init<>())
        .def(py::init<pf::path::arc2d::vector_t, pf::path::arc2d::vector_t, pf::path::arc2d::vector_t>(),
             py::arg("start"), py::arg("mid"), py::arg("end"))
        .def(py::init<pf::path::arc2d::vector_t, pf::path::arc2d::vector_t, pf::path::arc2d::vector_t,double,double>(),
             py::arg("start"), py::arg("mid"), py::arg("end"), py::arg("start_k"), py::arg("end_k"))
        .def("set_curvature", &pf::path::augmented_arc2d::set_curvature,
             py::arg("start_k"), py::arg("end_k"))
        .def("curvature", &pf::path::augmented_arc2d::curvature,
             py::arg("s"))
        .def("dcurvature", &pf::path::augmented_arc2d::dcurvature,
             py::arg("s"));
    
    path_augmented_arc2d.doc() = 
        "Implementation of @ref arc2d with non-constant curvature.\n"
        "\n"
        "The Augmented Arc is an arc with a non-continuous curvature, slightly disobeying\n"
        "the geometry of an arc segment. This class is used as the output of an approximation\n"
        "done by @ref arc_parameterizer designed to approximate splines with continous curvature.\n"
        "\n"
        "The curvature is interpolated with respect to the arc length of the segment. This is\n"
        "necessary for certain systems that require a parameterized spline.\n";

    // pf/path/curve.h

    auto path_curve = py::class_<pf::path::curve<2>>(path, "curve2")
        .def("position", &pf::path::curve::position,
             py::arg("s"))
        .def("derivative", &pf::path::curve::derivative,
             py::arg("s"))
        .def("rotation", &pf::path::curve::rotation,
             py::arg("s"))
        .def("curvature", &pf::path::curve::curvature,
             py::arg("s"))
        .def("dcurvature", &pf::path::curve::dcurvature,
             py::arg("s"))
        .def("length", &pf::path::curve::length);

    path_curve.doc() = 
        "A position curve parameterized to arc length 's'.\n"
        "\n"
        "A curve parameterized to its arc length, allowing extremely quick\n"
        "calculation of the length of the curve, primarily useful when using\n"
        "curves to generate a trajectory, as the parameter is a real-world unit\n"
        "relating directly to the state (distance travelled).\n";

    // pf/path/hermite.h

    // TODO
    // h2w ~/dev/frc/header2whatever/examples/pybind_cls.j2 pathfinder_src/Pathfinder/src/include/grpl/pf/path/hermite.h

    auto name = py::class_<grpl::pf::path::hermite, grpl::pf::path::spline<2>>(m, "hermite")
        .def(py::init<>())
        .def(py::init<control_matrix_t &>(),
             py::arg("M"))
        .def("set_control_matrix", &grpl::pf::path::hermite::set_control_matrix,
             py::arg("M"))
        .def("get_control_matrix", &grpl::pf::path::hermite::get_control_matrix)
        .def("position", &grpl::pf::path::hermite::position,
             py::arg("t"))
        .def("derivative", &grpl::pf::path::hermite::derivative,
             py::arg("t"))
        .def("derivative2", &grpl::pf::path::hermite::derivative2,
             py::arg("t"))
        .def("curvature", &grpl::pf::path::hermite::curvature,
             py::arg("t"))

    auto name = py::class_<grpl::pf::path::hermite_cubic>(m, "hermite_cubic")
        .def(py::init<>())
        .def(py::init<waypoint &, waypoint &>(),
             py::arg("start"), py::arg("end"))
        .def("set_waypoints", &grpl::pf::path::hermite_cubic::set_waypoints,
             py::arg("start"), py::arg("end"));

    auto name = py::class_<grpl::pf::path::hermite_cubic::waypoint>(m, "waypoint")
        .def_readwrite("position", &grpl::pf::path::hermite_cubic::waypoint::position)
        .def_readwrite("tangent", &grpl::pf::path::hermite_cubic::waypoint::tangent);

    auto name = py::class_<grpl::pf::path::hermite_quintic>(m, "hermite_quintic")
        .def(py::init<>())
        .def(py::init<waypoint &, waypoint &>(),
             py::arg("start"), py::arg("end"))
        .def("set_waypoints", &grpl::pf::path::hermite_quintic::set_waypoints,
             py::arg("start"), py::arg("end"));

    auto name = py::class_<grpl::pf::path::hermite_quintic::waypoint>(m, "waypoint")
        .def_readwrite("position", &grpl::pf::path::hermite_quintic::waypoint::position)
        .def_readwrite("tangent", &grpl::pf::path::hermite_quintic::waypoint::tangent)
        .def_readwrite("dtangent", &grpl::pf::path::hermite_quintic::waypoint::dtangent);

    // TODO: generator function, what's the best way to make this work without a lot of copies?

    // pf/path/spline.h

    auto path_spline = py::class_<grpl::pf::path::spline<2>>(m, "spline2")
        .def("position", &grpl::pf::path::spline::position,
             py::arg("t"))
        .def("derivative", &grpl::pf::path::spline::derivative,
             py::arg("t"))
        .def("rotation", &grpl::pf::path::spline::rotation,
             py::arg("t"))
        .def("curvature", &grpl::pf::path::spline::curvature,
             py::arg("t"));
    
    path_spline.doc() =
        "A position curve parameterized to spline parameter 't'.\n"
        "\n"
        "A spline is simply a position curve parameterized to spline parameter 't'\n"
        "(note that this is distinct to time), which lays in the range of 0 to 1,\n"
        "representing the start and end of the spline respectively.\n";

    //
    // pf/profile
    //

    auto profile = m.def_submodule("profile");

    // pf/profile/profile.h
    auto profile_state = py::class_<grpl::pf::profile::state>(profile, "state")
        .def_readwrite("time", &pf::profile::state::time)
        .def_readwrite("kinematics", &pf::profile::state::kinematics);
    
    profile_state.doc() = 
        "A single state (sample point) of a motion profile. This contains the kinematics of\n"
        "the profile sampled at a given time.\n"
        "\n"
        "The state kinematics goes up to a maximum order of @ref grpl::pf::constants::profile_kinematics_order,\n"
        "although implementations of @ref grpl::pf::profile::profile may or may not fill the entire vector,\n"
        "depending on their operating order (e.g. @ref grpl::pf::profile::trapezoidal will only fill up to\n"
        "@ref grpl::pf::ACCELERATION, all higher orders will be an undefined value).\n";

    auto profile_profile = py::class_<grpl::pf::profile::profile>(profile, "profile")
        .def("limited_term", &pf::profile::profile::limited_term)
        .def("set_goal", &pf::profile::profile::set_goal,
             py::arg("sp"))
        .def("get_goal", &pf::profile::profile::get_goal)
        .def("set_timeslice", &pf::profile::profile::set_timeslice,
             py::arg("timeslice"))
        .def("get_timeslice", &pf::profile::profile::get_timeslice)
        .def("apply_limit", &pf::profile::profile::apply_limit,
             py::arg("term"), py::arg("min"), py::arg("max"))
        .def("get_limits", &pf::profile::profile::get_limits)
        .def("calculate", &pf::profile::profile::calculate, release_gil(),
             py::arg("last"), py::arg("time"));
    
    profile_profile.doc() = 
        "Abstract base class for all motion profile types.\n"
        "\n"
        "A motion profile describes some function that forms the shape of the position-time curve\n"
        "and its derivatives, allowing velocity, acceleration and jerk to be shaped in ways that may\n"
        "be desirable, such as optimizing for speed, safety, or smooth operation.\n"
        "\n"
        "Profiles are predictive in Pathfinder, meaning each calculation does not require a full history\n"
        "of the profile. This allows it to adjust to changing system conditions and limits during a\n"
        "profile, increasing flexibility and efficiency.\n"
        "\n"
        "Since the system is predictive, it may result in a small oscillation or sudden deceleration\n"
        "if a sufficient timestep is not used. For this reason, a timeslice mechanism is included in the\n"
        "profile.\n";

    // pf/profile/trapezoidal.h
    auto profile_trapezoidal = py::class_<pf::profile::trapezoidal, pf::profile::profile>(profile, "trapezoidal")
        .def("limited_term", &pf::profile::trapezoidal::limited_term)
        .def("calculate", &pf::profile::trapezoidal::calculate,
             py::arg("last"), py::arg("time"), release_gil());
    
    profile_trapezoidal.doc() = 
        "Implementation of a trapezoidal (limited acceleration) motion profile.\n"
        "\n"
        "A trapezoidal motion profile is a motion profile limited by acceleration (therefore, infinite jerk).\n"
        "The profile can be described by three distinct sections: ramp-up, hold and ramp-down.\n"
        "\n"
        "During ramp-up, the system is accelerating towards its max velocity.\n"
        "\n"
        "During hold, the system is not accelerating, and holds its max velocity. Depending on the setpoint,\n"
        "the system may not have time to reach hold before it must ramp-down, resulting in a trianglular\n"
        "velocity profile.\n"
        "\n"
        "During ramp-down, the system is decelerating towards 0.\n"
        "\n"
        "See @ref grpl::pf::profile::profile\n";

    //
    // pf/transmission
    //

    auto tm = m.def_submodule("transmission");
    tm.doc() = 
        "Motor transmissions.\n"
        "\n"
        "The grpl::pf::transmission namespace contains implementations of transmissions, which convert\n"
        "control signals to motion (e.g. motors).\n";
    
    // pf/transmission/dc.h

    auto tm_dc_transmission = py::class_<pf::transmission::dc_transmission>(m, "dc_transmission")
        .def("get_free_speed", &pf::transmission::dc_transmission::get_free_speed,
                py::arg("voltage"))
        .def("get_current", &pf::transmission::dc_transmission::get_current,
                py::arg("voltage"), py::arg("speed"))
        .def("get_torque", &pf::transmission::dc_transmission::get_torque,
                py::arg("current"))
        .def("get_free_voltage", &pf::transmission::dc_transmission::get_free_voltage,
                py::arg("speed"))
        .def("get_current_voltage", &pf::transmission::dc_transmission::get_current_voltage,
                py::arg("current"))
        .def("get_torque_current", &pf::transmission::dc_transmission::get_torque_current,
                py::arg("torque"))
        .def("nominal_voltage", &pf::transmission::dc_transmission::nominal_voltage);

    tm_dc_transmission.doc() =
        "Abstract base class of a DC electric transmission, usually @ref grpl::pf::transmission::dc_motor.\n";
    
    auto tm_dc_motor = py::class_<pf::transmission::dc_motor>(m, "dc_motor")
        .def(py::init<double,double,double,double,double>(),
             py::arg("v_nom"), py::arg("free_speed"), py::arg("free_current"), py::arg("stall_current"), py::arg("stall_torque"))
        .def("internal_resistance", &pf::transmission::dc_motor::internal_resistance)
        .def("kv", &pf::transmission::dc_motor::kv)
        .def("kt", &pf::transmission::dc_motor::kt)
        .def("nominal_voltage", &pf::transmission::dc_motor::nominal_voltage)
        .def("get_current", &pf::transmission::dc_motor::get_current,
             py::arg("voltage"), py::arg("speed"))
        .def("get_torque", &pf::transmission::dc_motor::get_torque,
             py::arg("current"))
        .def("get_free_speed", &pf::transmission::dc_motor::get_free_speed,
             py::arg("voltage"))
        .def("get_free_voltage", &pf::transmission::dc_motor::get_free_voltage,
             py::arg("speed"))
        .def("get_current_voltage", &pf::transmission::dc_motor::get_current_voltage,
             py::arg("current"))
        .def("get_torque_current", &pf::transmission::dc_motor::get_torque_current,
             py::arg("torque"));

    tm_dc_motor.doc() = 
        "Mathematical Model of a DC Brushed Motor\n"
        "\n"
        "Basic DC Motor Model, dervied from the ideal resistive motor model with Back EMF\n"
        "(+ ---[ R ]---( V_w )--- -), where R = V / I_stall (as V_w = 0 at stall), and\n"
        "V_w = kv*w.\n";

}