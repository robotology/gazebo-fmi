# Simple electrical actuator example
This directory contains an example Gazebo world, that contains two one degrees of freedom manipulators.
The models are identical, with the difference that one (called `no_actuator_model`) uses the standard Gazebo actuation model,
while the other (called `simple_electrical_actuator_model`) is using a simple model of an electrical actuator, that is contained in the
`SimpleElectricalActuator.fmu` fmu, that is generated from the `SimpleElectricalActuator.mo` Modelica model.

## How to run
First  of all, make sure that you have `omc` (the OpenModelica compiler) in your PATH, otherwise
it will not be possible for the build system to generate an FMU from the `SimpleElectricalActuator.fmu` model.
After that, compile and install the `gazebo-fmi` project and source the `<install_prefix>/share/gazebo-fmi/examples/setup-examples.sh` bash script, where <install_prefix> is
your installation directory (the value of `CMAKE_INSTALL_PREFIX`).

At this point, you can launch the simulation using the following command:
~~~
gazebo simple_electical_actuator.world
~~~

The two models will start in the 0 position. You can change the target position of the internal Gazebo joint controllers using the `gz joint` command:
~~~
gz joint -m no_actuator_model -j joint --pos-p 10.0 --pos-d 3.0 --pos-t 1.0 && gz joint -m simple_electrical_actuator_model -j joint --pos-p 10.0 --pos-d 3.0 --pos-t 1.0
~~~
The `pos-t` command specify the target of the joint position controller, while `pos-p` and `pos-d` specifies the position and derivative gains of the internal Gazebo PID joint controller.

In this example it is possible to see that the gains that are able to stabilize with minimal overshoot and oscillations the `no_actuator_model` are not a good fit for the the `simple_electrical_actuator_model`, due to the use of the low-level eletrical actuator model

The idea is that in this way the Gazebo model, as long as the electical actuation is similar to the real model, can be used for the tuning (at least preliminary) of the gains of the real joint torque controllers.
