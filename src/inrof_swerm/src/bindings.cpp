#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "core/boid_core.hpp"
#include "core/follower_core.hpp"
#include "core/sheep_core.hpp"

PYBIND11_MODULE(inrof_swerm_cpp, m)
{
    m.doc() = "Python bindings for inrof_swerm";

    pybind11::class_<BoidPrams>(
        m, "BoidPrams",
        "Parameters controlling boid swarm velocity updates.")
        .def(pybind11::init<>(), "Create a boid parameter set.")
        .def_readwrite("boid_num", &BoidPrams::boid_num, "Number of boids in the swarm.")
        .def_readwrite("max_vel", &BoidPrams::max_vel, "Maximum boid velocity.")
        .def_readwrite("k_separation", &BoidPrams::k_separation, "Separation force gain.")
        .def_readwrite("k_alignment", &BoidPrams::k_alignment, "Alignment force gain.")
        .def_readwrite("k_gravity", &BoidPrams::k_gravity, "Cohesion force gain.")
        .def_readwrite("Ir", &BoidPrams::Ir, "Interaction radius.")
        .def_readwrite("Ir_min", &BoidPrams::Ir_min, "Minimum interaction radius.");
}
