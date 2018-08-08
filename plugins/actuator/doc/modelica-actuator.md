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
![nulltransmission](https://user-images.githubusercontent.com/1857049/43839218-d9c9960a-9b1d-11e8-8b2c-fbd712a0ff4a.png)

Modelica source code: [`NullTransmission.mo`](../test/NullTransmission.mo) .

This is the simplest example of transmission: regardless of the `actuatorInput` or the feedback coming from `jointPosition`, `jointVelocity` or `jointAcceleration`, the `jointTorque` output of the trasmission is always zero. In the unit test, this model
is used to verify that the PID is not able to control the joint position if this actuator is used.




### Identity Transmission
![identitytransmission](https://user-images.githubusercontent.com/1857049/43839217-d9a7acac-9b1d-11e8-8dba-e5548d3c874f.png)

Modelica source code: [`IdentityTransmission.mo`](../test/IdentityTransmission.mo) .


This is another simple example of transmission: regardless of the feedback coming from  `jointPosition`, `jointVelocity` or `jointAcceleration`, this model will always copy in `jointTorque` the `actuatorInput` value.

### Compliant Transmission
![complianttransmission](https://user-images.githubusercontent.com/1857049/43839216-d980bbc4-9b1d-11e8-81b4-eedfd46f4760.png)

Modelica source code: [`CompliantTransmission.mo`](../test/CompliantTransmission.mo) .

This is the first non-trivial example of the transmission: the actuator input is used to drive a series of an inertia and a spring,
that then drive the actual torque delivered to the Gazebo model. Note that, due to the spring between the inertia and the output flange,
the `jointTorque` output value does not depend on the `jointAcceleration` input.

### Soft Transmission
![softtransmission](https://user-images.githubusercontent.com/1857049/43839219-d9eb899a-9b1d-11e8-8475-9bd36f585d7c.png)

Modelica source code: [`SoftTransmission.mo`](../test/SoftTransmission.mo) .

This model is exactly the same of the Compliant Transmission one, but with the difference that the spring stiffness is much lower.
The same joint PID gains that are able to reach a given setpoint if the Compliant Transmission is used, are not able to control
the joint if this transmission is used, due to the low stiffness of the spring.

### Stiff Transmission
![stifftransmission](https://user-images.githubusercontent.com/1857049/43839220-da165d3c-9b1d-11e8-81b2-0ff04d149980.png)

Modelica source code: [`StiffTransmission.mo`](../test/StiffTransmission.mo) .

This model shows a transmission with a direct coupling between the transmission inertia and the output flange.
Note that, due to the direct coupling between the inertia and the output flange,
the `jointTorque` output value depends instantaneously on the `jointAcceleration` input.


