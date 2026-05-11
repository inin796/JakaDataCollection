#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "data_acquisition_algo.h"

namespace py = pybind11;

PYBIND11_MODULE(jaka_algo, m) {
    m.doc() = "JAKA Data Acquisition Algorithm (Pure C++, No API Calls)";

    py::class_<jaka_algo::RobotSample>(m, "RobotSample")
        .def(py::init<>())
        .def_readwrite("timestamp",     &jaka_algo::RobotSample::timestamp)
        .def_readwrite("joint_angles",  &jaka_algo::RobotSample::joint_angles)
        .def_readwrite("joint_torques", &jaka_algo::RobotSample::joint_torques)
        .def_readwrite("force_torque",  &jaka_algo::RobotSample::force_torque)
        .def_readwrite("joint_vel",     &jaka_algo::RobotSample::joint_vel)
        .def_readwrite("joint_acc",     &jaka_algo::RobotSample::joint_acc);

    py::class_<jaka_algo::AcquisitionConfig>(m, "AcquisitionConfig")
        .def(py::init<>())
        .def_readwrite("lower_limits",  &jaka_algo::AcquisitionConfig::lower_limits)
        .def_readwrite("upper_limits",  &jaka_algo::AcquisitionConfig::upper_limits)
        .def_readwrite("point_count",   &jaka_algo::AcquisitionConfig::point_count)
        .def_readwrite("vel_threshold", &jaka_algo::AcquisitionConfig::vel_threshold)
        .def_readwrite("acc_threshold", &jaka_algo::AcquisitionConfig::acc_threshold)
        .def_readwrite("stable_frames", &jaka_algo::AcquisitionConfig::stable_frames);

    py::class_<jaka_algo::DataAcquisitionAlgorithm>(m, "DataAcquisitionAlgorithm")
        .def(py::init<>())
        .def(py::init<const jaka_algo::AcquisitionConfig&>())
        .def("set_config",          &jaka_algo::DataAcquisitionAlgorithm::set_config)
        .def("get_config",          &jaka_algo::DataAcquisitionAlgorithm::get_config)
        .def("generate_random_points", &jaka_algo::DataAcquisitionAlgorithm::generate_random_points)
        .def("get_current_target",  &jaka_algo::DataAcquisitionAlgorithm::get_current_target)
        .def("has_next_target",     &jaka_algo::DataAcquisitionAlgorithm::has_next_target)
        .def("advance_target",      &jaka_algo::DataAcquisitionAlgorithm::advance_target)
        .def("feed_sensor_data",    &jaka_algo::DataAcquisitionAlgorithm::feed_sensor_data)
        .def("is_stationary",       &jaka_algo::DataAcquisitionAlgorithm::is_stationary)
        .def("get_collected_count", &jaka_algo::DataAcquisitionAlgorithm::get_collected_count)
        .def("export_csv",          &jaka_algo::DataAcquisitionAlgorithm::export_csv)
        .def("reset",               &jaka_algo::DataAcquisitionAlgorithm::reset);
}
