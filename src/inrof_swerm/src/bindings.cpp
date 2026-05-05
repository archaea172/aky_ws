#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "core/boid_core.hpp"
#include "core/follower_core.hpp"
#include "core/sheep_core.hpp"

PYBIND11_MODULE(_core, m)
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

    pybind11::class_<BoidCore>(
        m, "BoidCore",
        "Core boid swarm velocity updater.")
        .def(pybind11::init<const BoidPrams&>(), pybind11::arg("params"))
        .def(
            "update_vels",
            &BoidCore::update_vels,
            pybind11::arg("pos_matrix"),
            pybind11::arg("vel_matrix"),
            pybind11::call_guard<pybind11::gil_scoped_release>());

    pybind11::class_<FollowerCore, BoidCore>(
        m, "FollowerCore",
        "Boid swarm velocity updater with leader-following force.")
        .def(pybind11::init<const BoidPrams&, double>(), pybind11::arg("params"), pybind11::arg("k_follow"))
        .def(
            "update_vels",
            &FollowerCore::update_vels,
            pybind11::arg("pos_matrix"),
            pybind11::arg("vel_matrix"),
            pybind11::arg("leader_pos"),
            pybind11::call_guard<pybind11::gil_scoped_release>());

    pybind11::class_<SheepCore, BoidCore>(
        m, "SheepCore",
        "Boid swarm velocity updater with dog-avoidance force.")
        .def(pybind11::init<const BoidPrams&, double>(), pybind11::arg("params"), pybind11::arg("k_run"))
        .def(
            "update_vels",
            &SheepCore::update_vels,
            pybind11::arg("pos_matrix"),
            pybind11::arg("vel_matrix"),
            pybind11::arg("dog_pos"),
            pybind11::call_guard<pybind11::gil_scoped_release>());
}
