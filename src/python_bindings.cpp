
#include <pybind11/pybind11.h>

#include "simulator/Simulator.h"
#include "controller/Controller.h"

namespace py = pybind11;

PYBIND11_MODULE(rocket_sim, m) {
    m.doc() = "Python bindings for the BalloonInterceptor simulator";

    py::class_<State>(m, "State")
        .def_readwrite("target_x", &State::targetX)
        .def_readwrite("target_y", &State::targetY)
        .def_readwrite("rocket_x", &State::rocketX)
        .def_readwrite("rocket_y", &State::rocketY)
        .def_readwrite("square_distance", &State::squareDistance)
        .def_readwrite("rocket_vx", &State::rocketVx)
        .def_readwrite("rocket_vy", &State::rocketVy)
        .def_readwrite("target_vx", &State::targetVx)
        .def_readwrite("target_vy", &State::targetVy)
        .def_readwrite("orientation", &State::orientation)
        .def_readwrite("omega", &State::omega)
        .def_readwrite("thrust", &State::thrust)
        .def_readwrite("gimbal_angle", &State::gimbalAngle);

    py::class_<Action>(m, "Action")
        .def(py::init<double, double>(),
             py::arg("thrust"),
             py::arg("gimbal_angle"))
        .def_readwrite("thrust", &Action::_desiredThrust)
        .def_readwrite("gimbal_angle", &Action::_desiredAngle);

    py::class_<StepResult>(m, "StepResult")
        .def_readonly("state", &StepResult::state)
        .def_readonly("terminated", &StepResult::terminated);

    py::class_<Simulator>(m, "Simulator")
        .def(py::init<>())
        .def("initialise", &Simulator::initialise)
        .def("reset", &Simulator::reset)
        .def("get_state", &Simulator::getState)
        .def("step", &Simulator::step,
             py::arg("action"),
             py::arg("dt"));
}