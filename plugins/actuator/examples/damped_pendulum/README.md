# Damped Pendulum example
This directory contains a simple Gazebo world, that contains two damped pendulum models.
The models are identical, with the difference that in one the damping effect is computed
via the Gazebo's physics engine setting the `damping` SDF tag, while for the other the damping
is computed via the Modelica model `Damper.mo`.

## How to run
First  of all, make sure that you have `omc` (the OpenModelica compiler) in your PATH, otherwise
it will not be possible for the build system to generate an FMU from the `Damper.fmu` model.
After that, compile and install the `gazebo-fmi` project and source the `<install_prefix>/share/gazebo-fmi/examples/setup-examples.sh` bash script, where <install_prefix> is
your installation directory (the value of `CMAKE_INSTALL_PREFIX`).

At this point, you can launch the simulation of the damped pendulum using the following command:
~~~
gazebo -u -e simbody damped_pendulum.world
~~~
the `-u` option ensures that the simulation start paused, while the `-e simbody` that the simulation
is performed with simbody, the most accurate of the physics engine available by default in Gazebo.
If you play the simulation, you will  see how the movements of the two pendulum will be synchronized, even
if one is simulating the damping via the Gazebo's physics engine, and one via the damping model contained  in `Damper.mo`.
