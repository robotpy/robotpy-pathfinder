#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Use this to release the gil
typedef py::call_guard<py::gil_scoped_release> release_gil;

#include "pathfinder.h"

//
// Wrappers around some pathfinder functions that modify their arguments,
// as pybind11 does not allow pass-by-reference lists of objects
//

typedef std::vector<Segment> Trajectory;

static bool _pathfinder_serialize(const char * fname, Trajectory trajectory) {
    FILE * fp = fopen(fname, "w");
    if (fp == NULL) {
        return false;
    }
    pathfinder_serialize(fp, trajectory.data(), trajectory.size());
    fclose(fp);
    return true;
}

static Trajectory _pathfinder_deserialize(const char * fname) {
    Trajectory ret;
    FILE * fp = fopen(fname, "r");
    if (fp != NULL) {
        
        // find out how many records are going to be read
        char buf_1[4];
        fread(buf_1, 1, 4, fp);
        int length = bytesToInt(buf_1);
        
        fseek(fp, 0, SEEK_SET);
        ret.resize(length);
        
        pathfinder_deserialize(fp, ret.data());
        fclose(fp);
    }
    return ret;
}

static bool _pathfinder_serialize_csv(const char * fname, Trajectory trajectory) {
    FILE * fp = fopen(fname, "w");
    if (fp == NULL) {
        return false;
    }
    pathfinder_serialize_csv(fp, trajectory.data(), trajectory.size());
    fclose(fp);
    return true;
}

static std::tuple<TrajectoryInfo, Trajectory> _generate(
    std::vector<Waypoint> path, void* fit,
    int sample_count, double dt, double max_velocity, double max_acceleration, double max_jerk) {
    
    TrajectoryCandidate cd;
    if (pathfinder_prepare(path.data(), path.size(), (void (*)(Waypoint,Waypoint,Spline*))fit, sample_count, dt, max_velocity, max_acceleration, max_jerk, &cd) < 0) {
        throw new std::invalid_argument("The path is not long enough");
    }
    
    Trajectory segs(cd.length);
    if (pathfinder_generate(&cd, segs.data()) < 0) {
        throw new std::invalid_argument("The trajectory provided was invalid! Invalid trajectory could not be generated");
    }
    
    return std::make_tuple(std::move(cd.info), std::move(segs));
}

static std::tuple<Trajectory, Trajectory, Trajectory, Trajectory>
       _pathfinder_modify_swerve(Trajectory &source,double wheelbase_width, double wheelbase_depth) {
    Trajectory fl(source.size());
    Trajectory fr(source.size());
    Trajectory bl(source.size());
    Trajectory br(source.size());
    
    pathfinder_modify_swerve(source.data(), source.size(), fl.data(), fr.data(), bl.data(), br.data(), wheelbase_width, wheelbase_depth, SWERVE_DEFAULT);
    return std::make_tuple(std::move(fl), std::move(fr), std::move(bl), std::move(br));
}

static std::tuple<Trajectory, Trajectory> _pathfinder_modify_tank(Trajectory &source, double wheelbase_width) {
    Trajectory left(source.size());
    Trajectory right(source.size());
    pathfinder_modify_tank(source.data(), source.size(), left.data(), right.data(), wheelbase_width);
    return std::make_tuple(std::move(left), std::move(right));
}


//
// Module declaration
//

#define REPR(x) " " #x "=" + std::to_string(__inst.x)

PYBIND11_MODULE(_pathfinder, m) {

    m.attr("__version__") = VERSION_INFO;
    
    // fit.h
    // TODO: these should be real functions, but had issues
    m.attr("pf_fit_hermite_pre") = (void*)&pf_fit_hermite_pre;
    m.attr("pf_fit_hermite_cubic") = (void*)&pf_fit_hermite_cubic;
    m.attr("pf_fit_hermite_quintic") = (void*)&pf_fit_hermite_quintic;
    
    // io.h
    
    // spline.h
    m.attr("SAMPLES_FAST") = PATHFINDER_SAMPLES_FAST;
    m.attr("SAMPLES_LOW") = PATHFINDER_SAMPLES_LOW;
    m.attr("SAMPLES_HIGH") = PATHFINDER_SAMPLES_HIGH;
    
    // structs.h
    py::class_<Waypoint>(m, "Waypoint")
        .def(py::init<>())
        .def(py::init<double, double, double>())
        .def_readwrite("x", &Waypoint::x)
        .def_readwrite("y", &Waypoint::y)
        .def_readwrite("angle", &Waypoint::angle)
        .def("__repr__", [](Waypoint &__inst) {
            return std::string("<Waypoint") + REPR(x) + REPR(y) + REPR(angle) + ">";
        });
    
    py::class_<Spline>(m, "Spline")
        .def(py::init<>())
        .def_readwrite("a", &Spline::a)
        .def_readwrite("b", &Spline::b)
        .def_readwrite("c", &Spline::c)
        .def_readwrite("d", &Spline::d)
        .def_readwrite("e", &Spline::e)
        .def_readwrite("x_offset", &Spline::x_offset)
        .def_readwrite("y_offset", &Spline::y_offset)
        .def_readwrite("angle_offset", &Spline::angle_offset)
        .def_readwrite("knot_distance", &Spline::knot_distance)
        .def_readwrite("arc_length", &Spline::arc_length)
        // methods
        .def("angle", &pf_spline_angle)
        .def("coords", &pf_spline_coords)
        .def("deriv", &pf_spline_deriv)
        .def("deriv_2", &pf_spline_deriv_2)
        .def("distance", &pf_spline_distance)
        .def("progress_for_distance", &pf_spline_progress_for_distance);
    
    py::class_<Coord>(m, "Coord")
        .def(py::init<>())
        .def_readwrite("x", &Coord::x)
        .def_readwrite("y", &Coord::y)
        .def("__repr__", [](Coord &__inst) {
            return std::string("<Coord") + REPR(x) + REPR(y) + ">";
        });
    
    py::class_<Segment>(m, "Segment")
        .def(py::init<>())
        .def_readwrite("dt", &Segment::dt)
        .def_readwrite("x", &Segment::x)
        .def_readwrite("y", &Segment::y)
        .def_readwrite("position", &Segment::position)
        .def_readwrite("velocity", &Segment::velocity)
        .def_readwrite("acceleration", &Segment::acceleration)
        .def_readwrite("jerk", &Segment::jerk)
        .def_readwrite("heading", &Segment::heading)
        .def("__repr__", [](Segment &__inst) {
            return std::string("<Segment") + REPR(dt) + REPR(x) + REPR(y) + REPR(position) + REPR(velocity) + REPR(acceleration) + REPR(jerk) + REPR(heading) + ">";
        });
    
    /*py::class_<TrajectoryConfig>(m, "TrajectoryConfig")
        .def(py::init<>())
        .def_readwrite("dt", &TrajectoryConfig::dt)
        .def_readwrite("max_velocity", &TrajectoryConfig::max_v)
        .def_readwrite("max_acceleration", &TrajectoryConfig::max_a)
        .def_readwrite("max_jerk", &TrajectoryConfig::max_j)
        .def_readwrite("src_v", &TrajectoryConfig::src_v)
        .def_readwrite("src_theta", &TrajectoryConfig::src_theta)
        .def_readwrite("dest_pos", &TrajectoryConfig::dest_pos)
        .def_readwrite("dest_v", &TrajectoryConfig::dest_v)
        .def_readwrite("dest_theta", &TrajectoryConfig::dest_theta)
        .def_readwrite("sample_count", &TrajectoryConfig::sample_count);*/
    
    py::class_<TrajectoryInfo>(m, "TrajectoryInfo")
        .def(py::init<>())
        .def_readwrite("filter1", &TrajectoryInfo::filter1)
        .def_readwrite("filter2", &TrajectoryInfo::filter2)
        .def_readwrite("length", &TrajectoryInfo::length)
        .def_readwrite("dt", &TrajectoryInfo::dt)
        .def_readwrite("u", &TrajectoryInfo::u)
        .def_readwrite("v", &TrajectoryInfo::v)
        .def_readwrite("impulse", &TrajectoryInfo::impulse)
        .def("__repr__", [](TrajectoryInfo &__inst) {
            return std::string("<TrajectoryInfo") + REPR(filter1) + REPR(filter2) + REPR(length) + REPR(dt) + REPR(u) + REPR(v) + REPR(impulse) + ">";
        });
    
    // trajectory.h
    //m.def("pathfinder_prepare", &pathfinder_prepare);
    //m.def("pathfinder_generate", &_pathfinder_generate);
    
    //m.def("pf_trajectory_prepare", &pf_trajectory_prepare);
    //m.def("pf_trajectory_create", &pf_trajectory_create);
    
    // followers/distance.h
    py::class_<FollowerConfig>(m, "FollowerConfig")
        .def(py::init<>())
        .def_readwrite("kp", &FollowerConfig::kp)
        .def_readwrite("ki", &FollowerConfig::ki)
        .def_readwrite("kd", &FollowerConfig::kd)
        .def_readwrite("kv", &FollowerConfig::kv)
        .def_readwrite("ka", &FollowerConfig::ka);
    
    py::class_<DistanceFollower>(m, "DistanceFollower")
        .def(py::init<>())
        .def_readwrite("last_error", &DistanceFollower::last_error)
        .def_readwrite("heading", &DistanceFollower::heading)
        .def_readwrite("output", &DistanceFollower::output)
        .def_readwrite("segment", &DistanceFollower::segment)
        .def_readwrite("finished", &DistanceFollower::finished);
    
    // followers/encoder.h
    py::class_<EncoderConfig>(m, "EncoderConfig")
        .def(py::init<>())
        .def_readwrite("initial_position", &EncoderConfig::initial_position)
        .def_readwrite("ticks_per_revolution", &EncoderConfig::ticks_per_revolution)
        .def_readwrite("wheel_circumference", &EncoderConfig::wheel_circumference)
        .def_readwrite("kp", &EncoderConfig::kp)
        .def_readwrite("ki", &EncoderConfig::ki)
        .def_readwrite("kd", &EncoderConfig::kd)
        .def_readwrite("kv", &EncoderConfig::kv)
        .def_readwrite("ka", &EncoderConfig::ka);
    
    py::class_<EncoderFollower>(m, "EncoderFollower")
        .def(py::init<>())
        .def_readwrite("last_error", &EncoderFollower::last_error)
        .def_readwrite("heading", &EncoderFollower::heading)
        .def_readwrite("output", &EncoderFollower::output)
        .def_readwrite("segment", &EncoderFollower::segment)
        .def_readwrite("finished", &EncoderFollower::finished);
    
    m.def("pathfinder_generate", &_generate, release_gil(),
        py::arg("path"), py::arg("fit"), py::arg("sample_count"),
        py::arg("dt"), py::arg("max_velocity"), py::arg("max_acceleration"),
        py::arg("max_jerk"),
        
        "Generate a motion profile trajectory using the given waypoints and configuration.\n"
        "\n"
        ":param path: A list of waypoints (setpoints) for the trajectory path to intersect\n"
        ":param fit: A fit function; use FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC\n"
        ":param sample_count:\n"
        ":param dt:\n"
        ":param max_velocity:\n"
        ":param max_acceleration:\n"
        ":param max_jerk:\n"
        "\n"
        ":returns: A tuple of TrajectoryInfo, and a generated trajectory (a list of segments)\n");
    
    m.def("pathfinder_follow_distance2", &pathfinder_follow_distance2, release_gil());
    m.def("pathfinder_follow_encoder2", &pathfinder_follow_encoder2, release_gil());
    
    m.def("pathfinder_modify_swerve", &_pathfinder_modify_swerve, release_gil());
    m.def("pathfinder_modify_tank", &_pathfinder_modify_tank, release_gil());
    
    m.def("pathfinder_deserialize", &_pathfinder_deserialize, release_gil(), py::arg("fname"));
    m.def("pathfinder_serialize", &_pathfinder_serialize, release_gil(),
          py::arg("fname"), py::arg("trajectory"));
    
    m.def("pathfinder_serialize_csv", &_pathfinder_serialize_csv, release_gil(),
          py::arg("fname"), py::arg("trajectory"));
}
