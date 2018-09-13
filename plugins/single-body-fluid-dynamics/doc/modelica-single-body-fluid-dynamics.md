# Use of the gazebo-fmi Single Body Fluid Dynamics plugin with Modelica

**Note: this document discusses the use Modelica with the gazebo-fmi Single Body Fluid Dynamics plugin, using [OpenModelica](https://openmodelica.org/). The content should apply also to other Modelica implementation that support the FMI 2.0 standard, but their use was not tested.**

## Generating FMU from a Modelica model on OpenModelica
There are two ways to generate the FMU file in OpenModelica. Be sure to choose the correct parameters.

### GUI
Execute OpenModelica and load your model. Eventually load all the needed libraries (i.e. Modelica_Synchronous).
Open the menu **Tools->Options**, tab FMI now choose the correct parameters:

**Version:2  
Type:Co-Simulation  
Platform:Dynamics**

Open the menu **FMI->Export FMU** at this point you have generated the .fmu file.

### Command line
Execute OpenModelica and load your model. Eventually load all the needed libraries (i.e. Modelica_Synchronous).
Open the menu **Tools->OpenModelica compiler CLI**
 and type

```
buildModelFMU(<Your model name without .mo>, "2", "cs", "<default>", {"dynamic"}, true)
```

At this point you have generated the .fmu file.

## Examples models 
In this repo, several examples of Modelica models are used for testing purpouses. We present each of this model as an example of Modelica models used to generate FMU suitable for the single body fluid dynamics forces co-simulation.


### Null Friction
![nullfriction](https://user-images.githubusercontent.com/1857049/46574162-be085400-c99f-11e8-8662-85dd2a1c41ca.png)

Modelica source code: [`NullFriction.mo`](../test/NullFriction.mo) .

This is the simplest example of fluid force: regardless of the value of  `relativeVelocity`,  the fluid dynamics force and moment are always equal to zero. This is equivalent to not load the plugin at all, and is used for consistency tests.

### Linear Friction
![linearfriction](https://user-images.githubusercontent.com/1857049/46574161-be085400-c99f-11e8-868e-c6327a43ff7c.png)

Modelica source code: [`LinearFriction.mo`](../test/LinearFriction.mo) .

This is a simple (and not realistic at all) friction, in  which the fluid force is linearly related to the relative velocity between the body and the fluid.



