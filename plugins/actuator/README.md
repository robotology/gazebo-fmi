# gazebo-fmi Actuator plugin 

**Note: for instructions on how to install the repo, please check [gazebo-fmi main README](../../README.md).**

The Gazebo FMI actuator plugin is a [Gazebo Model plugin](http://gazebosim.org/tutorials?tut=plugins_model) that enables to use any tool that supports the [FMI for Co-Simulation standard v2.0](https://fmi-standard.org/), to simulate
the actuator dynamics of a Gazebo model.

## Use the plugin 
Example configuration: 
~~~
<model>
  ...
  <plugin name="fmi_actuator_plugin" filename="libFMIActuatorPlugin.so">
    <actuator>
      <name>actuator_0</name> 
      <joint>JOINT_0</joint> 
      <fmu>electric_motor</fmu>
      <actuatorInputName>actuatorInput</actuatorInputName>
      <jointPositionName>jointPosition</jointPositionName>
      <jointVelocityName>jointVelocity</jointVelocityName>
      <jointTorqueName>jointTorque</jointTorqueName>
    </actuator>
   </plugin>
</model>
~~~
The plugin filename is `libFMIActuatorPlugin.so` .

Documentation of the parameters of the `<actuator>` tag. All the parameters are required

| Parameter name | Type    | Description                 | Notes |
|:--------------:|:-------:|:--------------------------: |:-----:|
| name           | string  | Name of the actuator, used for printing debug and error messages. |  |
| joint          | string  | Name of the joint. | The total list of joints contained in the model is scanned and the first joint that **ends** with this  name string is found. This is done to easily support nested models. Alternatively you can specify directly the **scoped joint name** as well.   |
| fmu            | string  | Filename of the FMU plugin to use for actuator co-simulation. | This name is passed to the [`gazebo::common::SystemPaths::FindFile`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1common_1_1SystemPaths.html#a9e03f07eac9f89d8c4c14af5660fa938) method to find the absolute location of the FMU file. Adding the directory containing the FMUs to the [`GAZEBO_RESOURCE_PATH`](http://gazebosim.org/tutorials?tut=components) should be sufficient to make it visible to the plugin. |
| actuatorInputName | string | Name of the FMU input variable representing the actuator input. | This variable should be present in the FMU with causality `input`. | 
| jointPositionName | string | Name of the FMU input variable representing the joint position. | This variable should be present in the FMU with causality `input`. | 
| jointVelocityName | string | Name of the FMU input variable representing the joint velocity. | This variable should be present in the FMU with causality `input`. | 
| jointTorqueName   | string | Name of the FMU output variable representing the joint torque. | This variable should be present in the FMU with causality `output`. | 


For a precise definition of what the `causality` of a FMU variable is, see Section 3.2 of [the "Functional Mockup Interface 2.0: The Standard for Tool independent Exchange of Simulation Models" paper](http://lup.lub.lu.se/search/ws/files/5428900/2972293.pdf) or 
Section 2.2.7, Page 46 of the ["Functional Mock-up Interface for
Model Exchange and Co-Simulation" v2.0 specification](https://svn.modelica.org/fmi/branches/public/specifications/v2.0/FMI_for_ModelExchange_and_CoSimulation_v2.0.pdf).

An example related to co-simulation of force elements (i.e. an actuator model without `actuatorInput`)  is described in Section 4.1 of [the "Functional Mockup Interface 2.0: The Standard for Tool independent Exchange of Simulation Models" paper](http://lup.lub.lu.se/search/ws/files/5428900/2972293.pdf). 

## Operating principle
The `gazebo-fmi` actuator plugin is able to be used transparently with the joint of any Gazebo model, regardless of how the joint is controlled: using the [Gazebo internal PID controller](http://gazebosim.org/tutorials?tut=guided_i5) or using middleware specific control plugin such as [`gazebo_ros_pkgs`](http://wiki.ros.org/gazebo_ros_pkgs) or [`gazebo-yarp-plugins`](https://github.com/robotology/gazebo-yarp-plugins).

All this software control the behaviour of the joints by calling the [`Joint::SetForce`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1physics_1_1Joint.html#ab2491053d1c5ebb97c377064797af494) in the main Gazebo thread, as part of the callback of the [`worldUpdateBegin`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1event_1_1Events.html#a78186ba279aac53f069f74143c53a4e4).

The `gazebo-fmi` actuator plugin code runs **after** the joint control code has run in the `worldUpdateBegin` event, but **before** the actual physics engine is updated.
This is achieved by running in the callback of the [`beforePhysicsUpdate`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1event_1_1Events.html#a18e51f6dcd556597aa4fcd35aaa4b8b2) event. The actuator plugins reads the value that was set in the `Joint::SetForce` method,
and it passes it to the FMU as the `actuatorInput` . It then runs the FMU simulation, and it substitus the value that was set in the `Joint::SetForce` with the
`jointTorque` output of the FMU. For more information, this logic can be found in the [`FMIActuatorPlugin::BeforePhysicsUpdateCallback`](https://github.com/robotology-playground/gazebo-fmi/blob/master/plugins/actuator/FMIActuatorPlugin.cc#L203) method.

For more info about the Gazebo events sequence, you can check the source code of Gazebo, in the [`gazebo::physics::World::Update()`](https://bitbucket.org/osrf/gazebo/src/01c7f8b1d68448bc618b575ad1c7ec13fee2b87f/gazebo/physics/World.cc#lines-746) method.


## Modelica examples
The gazebo-fmi plugins can be used with any standard-compliant FMU.
A common language used for generating actuator or trasmission models in the form of FMUs is [Modelica](https://www.modelica.org/), that is
an object-oriented, equation based language to conveniently model complex physical systems. Modelica is used in several  Simulation Environments, such as [OpenModelica](https://openmodelica.org/) or [Dymola](https://www.3ds.com/products-services/catia/products/dymola).
Please see the [`doc/modelica-actuator.md`](doc/modelica-actuator.md) document for additional insight on how to use the gazebo-fmi Actuator plugin with Modelica models.





