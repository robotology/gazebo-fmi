# gazebo-fmi Single Body Fluid Dynamics forces plugin

**Note: for instructions on how to install the repo, please check [gazebo-fmi main README](../../README.md).**

The Gazebo FMI Single Body  Fluid Dynamics plugin is a [Gazebo Model plugin](http://gazebosim.org/tutorials?tut=plugins_model) that enables to use any tool that supports the [FMI for Co-Simulation standard v2.0](https://fmi-standard.org/), to simulate the interaction forces between an isolated body and the surronding fluid, as function 
of the relative velocity between the body and the fluid. 
As it is possible to understand from the name, this plugin does not permit to simulate the mutual interaction of two or more bodies close to  each other with the surronding medium.

## Use the plugin 
Example configuration: 
~~~
<model>
  ...
  <plugin name="fmi_aero_plugin" filename="libFMISingleBodyFluidDynamicsPlugin.so">
    <single_body_fluid_fynamics>
      <name>drag_0</name> 
      <link>link0</link>
      <fmu>drag.fmu</fmu>
    </single_body_fluid_fynamics>
  </plugin>
</model>
~~~
The plugin filename is `libFMISingleBodyFkuidDynamicsPlugin.so` .

Documentation of the parameters of the `<single_body_fluid_dynamics>` tag. All the parameters are required

| Parameter name | Type    | Description                 | Notes |
|:--------------:|:-------:|:--------------------------: |:-----:|
| name           | string  | Name of the actuator, used for printing debug and error messages. |  |
| link          | string  | Name of the link. | The total list of joints contained in the model is scanned and the first joint that **link** with this  name string is found. This is done to easily support nested models. Alternatively you can specify directly the **scoped link name** as well.   |
| fmu            | string  | Filename of the FMU plugin to use for actuator co-simulation. | This name is passed to the [`gazebo::common::SystemPaths::FindFile`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1common_1_1SystemPaths.html#a9e03f07eac9f89d8c4c14af5660fa938) method to find the absolute location of the FMU file. Adding the directory containing the FMUs to the [`GAZEBO_RESOURCE_PATH`](http://gazebosim.org/tutorials?tut=components) should be sufficient to make it visible to the plugin. |
| variable_names            | string  | Filename of the FMU plugin to use for actuator co-simulation. | This name is passed to the [`gazebo::common::SystemPaths::FindFile`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1common_1_1SystemPaths.html#a9e03f07eac9f89d8c4c14af5660fa938) method to find the absolute location of the FMU file. Adding the directory containing the FMUs to the [`GAZEBO_RESOURCE_PATH`](http://gazebosim.org/tutorials?tut=components) should be sufficient to make it visible to the plugin. |


Variables:
| Parameter name | Type    | Description                 | Notes |
|:--------------:|:-------:|:--------------------------: |:-----:|
| `relativeVelocity_x` | X component of the relative velocity between the body and the medium, expressed in the body frame. | This variable should be present in the FMU with causality `input`, and is expressed in m/s. |
| `relativeVelocity_y` | Y component of the relative velocity between the body and the medium, expressed in the body frame. | This variable should be present in the FMU with causality `input`, and is expressed in m/s. |
| `relativeVelocity_z` | Z component of the relative velocity between the body and the medium, expressed in the body frame. | This variable should be present in the FMU with causality `input`, and is expressed in m/s. |
| `fluidDynamicForce_x`   | X component of the aerodynamic force exerted by the medium on body, expressed in the body frame. | This variable should be present in the FMU with causality `output`,  and  is expressed in N. |
| `fluidDynamicForce_y`   | Y component of the aerodynamic force exerted by the medium on body, expressed in the body frame. | This variable should be present in the FMU with causality `output`, and  is expressed in N. |
| `fluidDynamicForce_z`   | Z component of the aerodynamic force exerted by the medium on body, expressed in the body frame. | This variable should be present in the FMU with causality `output`,  and  is expressed in N. |
| `fluidDynamicMoment_x`   | X component of the aerodynamic moment exerted by the medium on body, expressed in the body orientation and w.r.t. to the boy origin. | This variable should be present in the FMU with causality `output`, and  is expressed in Nm. |
| `fluidDynamicMoment_y`   | Y component ofthe aerodynamic moment exerted by the medium on body, expressed in the  body orientation and w.r.t. to the boy origin. | This variable should be present in the FMU with causality `output`, and  is expressed in Nm. |
| `fluidDynamicMoment_z`   | Z component of the aerodynamic moment exerted by the medium on body, expressed in the  body orientation and w.r.t. to the boy origin.| This variable should be present in the FMU with causality `output`, and  is expressed in Nm. |

For a precise definition of what the `causality` of a FMU variable is, see Section 3.2 of [the "Functional Mockup Interface 2.0: The Standard for Tool independent Exchange of Simulation Models" paper](http://lup.lub.lu.se/search/ws/files/5428900/2972293.pdf) or [Section 2.2.7 of the "Functional Mock-up Interface for
Model Exchange and Co-Simulation" v2.0 specification](https://fmi-standard.org/docs/2.0.1-develop/#_definition_of_model_variables_modelvariables).


## Operating principle
The `gazebo-fmi` single body fluid dynamics plugin code is called during in the `worldUpdateBegin` event.
The plugin reads the velocity of the body using and the velocity of the wind, and computes from this the relative velocity.

The working of the plugin was inspired by the Gazebo plugin LiftDragPlugin, described in the [Gazebo's Aerodynamics tutorial](http://gazebosim.org/tutorials?tut=aerodynamics&cat=plugins).

## Modelica examples
The gazebo-fmi plugins can be used with any standard-compliant FMU.
For testing purpouses we generate fluid dynamics FMUs with [Modelica](https://www.modelica.org/), that is
an object-oriented, equation based language to conveniently model complex physical systems. Modelica is used in several  Simulation Environments, such as [OpenModelica](https://openmodelica.org/) or [Dymola](https://www.3ds.com/products-services/catia/products/dymola).
Please see the [`doc/modelica-single-body-fluid-dynamics.md`](doc/modelica-single-body-fluid-dynamics.md) document for additional insight on how to use the gazebo-fmi Single Body Fluid Dynamics plugin with Modelica models.






