# gazebo-fmi Actuator plugin 

**Note: for instructions on how to install the repo, please check [gazebo-fmi main README](../../README.md).**

The Gazebo FMI actuator is a [Gazebo Model plugin](http://gazebosim.org/tutorials?tut=plugins_model) that enables to use any tool that supports the [FMI for Co-Simulation standard v2.0](https://fmi-standard.org/), to simulate 
the actuator dynamics of a Gazebo model.

## Use the plugin 
Example configuration: 
~~~
<model>
  ...
  <plugin name="fmi_actuator_plugin" filename="libFMIActuatorPlugin.so">
    <actuator>
      <name>actuator_0</name> <!-- Name of the  -->
      <joint>JOINT_0</joint> <!-- name of joint to actuate in the model -->
      <fmu>electric_motor</fmu> <!-- name of the fmu -->
      <actuatorInputName>actuatorInput</actuatorInputName> <!-- FMU actuator input variable name -->
      <jointPositionName>jointPosition</jointPositionName> <!-- FMU joint position input variable name -->
      <jointVelocityName>jointVelocity</jointVelocityName> <!-- FMU joint velocity input variable name -->
      <jointTorqueName>jointTorque</jointTorqueName>       <!-- FMU joint torque output variable name -->
    </actuator>
   </plugin>
</model>
~~~
The plugin filename is `libFMIActuatorPlugin.so` .

Documentation of the parameters of the `<actuator>` tag. All the parameters are required

| Parameter name | Type    | Description                 | Notes |
|:--------------:|:-------:|:--------------------------: |:-----:|
| name           | string  | Name of the actuator, used for printing debug and error messages. |  |
| joint          | string  | Name of the joint. | The total list of joints contained in the model is scanned and the first joint that **ends** with this  name string is found. This is done to easily support nested models.  |
| fmu            | string  | Filename of the FMU plugin to use for actuator co-simulation. | This name is passed to the [`gazebo::common::SystemPaths::FindFile`](http://osrf-distributions.s3.amazonaws.com/gazebo/api/9.0.0/classgazebo_1_1common_1_1SystemPaths.html#a9e03f07eac9f89d8c4c14af5660fa938) method to find the absolute location of the FMU file. Adding the directory containing the FMUs to the [`GAZEBO_RESOURCE_PATH`](http://gazebosim.org/tutorials?tut=components) should be sufficient to make it visible to the plugin. |
| actuatorInputName | string | Name of the FMU input variable representing the actuator input. | This variable should be present in the FMU with causality INPUT. | 
| jointPositionName | string | Name of the FMU input variable representing the joint position. | This variable should be present in the FMU with causality INPUT. | 
| jointVelocityName | string | Name of the FMU input variable representing the joint velocity. | This variable should be present in the FMU with causality INPUT. | 
| jointTorqueName   | string | Name of the FMU output variable representing the joint torque. | This variable should be present in the FMU with causality OUTPUT. | 



