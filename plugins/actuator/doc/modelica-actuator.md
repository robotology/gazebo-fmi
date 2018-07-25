# Use of the gazebo-fmi Actuator plugin with Modelica

**Note: this document discusses the use Modelica with the gazebo-fmi Actuator plugin, using [OpenModelica](https://openmodelica.org/). The content should apply also to other Modelica implementation that support the FMI 2.0 standard, but their use was not tested.**

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
In this repo, several examples of Modelica models are used for testing purpouses. We present each of this model as an example of Modelica models used to generate FMU suitable for the actuator co-simulation.

All models use the [`Modelica.Mechanics.Rotational.Components.AngleToTorqueAdaptor`](https://doc.modelica.org/help/Modelica_Mechanics_Rotational_Components.html#Modelica.Mechanics.Rotational.Components.AngleToTorqueAdaptor) component from
the [Modelica Standard Library](https://github.com/modelica/ModelicaStandardLibrary) to map the [acausal connectors of Modelica](http://book.xogeny.com/components/connectors/)
to the input-output variables of the FMUs.

To read more about these interfaces between the acausal world of Modelica and the causal world of FMI, check:
* https://www.claytex.com/tech-blog/splitting-mechanical-and-fluid-devices-using-real-inputs-real-outputs/
* [Martin Otter - FMI Tutorial - Connecting tightly coupled FMUs](https://github.com/robotology-playground/gazebo-fmi/files/2195839/11_TigtlyConnectedFMUs.pdf)


### Null Transmission
![nulltransmission](https://user-images.githubusercontent.com/1857049/42735230-afa799ba-8850-11e8-875e-7e9952be0b3e.png)

Modelica source code: [`NullTransmission.mo`](../test/NullTransmission.mo) .

This is the simplest example of transmission: regardless of the `actuatorInput` or the feedback coming from `jointPosition`
or `jointVelocity`, the `jointTorque` output of the trasmission is always zero. In the unit test, this model
is used to verify that the PID is not able to control the joint position if this actuator is used.




### Identity Transmission
![identitytransmission](https://user-images.githubusercontent.com/1857049/42735229-af8172e4-8850-11e8-9d40-5674802087e7.png)

Modelica source code: [`IdentityTransmission.mo`](../test/IdentityTransmission.mo) .


This is another simple example of transmission: regardless of the feedback coming from `jointPosition`
or `jointVelocity`, this model will always copy in `jointTorque` the `actuatorInput` value.

### Compliant Transmission
![complianttransmission](https://user-images.githubusercontent.com/1857049/42735228-af5bf096-8850-11e8-9093-3e2ee4a49b80.png)

Modelica source code: [`CompliantTransmission.mo`](../test/CompliantTransmission.mo) .

This is the first non-trivial example of the transmission: the actuator input is used to drive a series of an inertia and a spring,
that then drive the actual torque delivered to the Gazebo model.

### Soft Transmission
![softtransmission](https://user-images.githubusercontent.com/1857049/42735226-ae968b94-8850-11e8-96e3-ec2ac2cb665d.png)

Modelica source code: [`SoftTransmission.mo`](../test/SoftTransmission.mo) .

This model is exactly the same of the Compliant Transmission one, but with the difference that the spring stiffness is much lower.
The same joint PID gains that are able to reach a given setpoint if the Compliant Transmission is used, are not able to control
the joint if this transmission is used, due to the low stiffness of the spring.

