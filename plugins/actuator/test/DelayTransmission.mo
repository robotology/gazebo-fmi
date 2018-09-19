model DelayTransmission
  Modelica.Blocks.Interfaces.RealInput actuatorInput annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput jointPosition annotation(
    Placement(visible = true, transformation(origin = {120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput jointVelocity annotation(
    Placement(visible = true, transformation(origin = {120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealOutput jointTorque annotation(
    Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.Torque torque1 annotation(
    Placement(visible = true, transformation(origin = {-8, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.AngleToTorqueAdaptor angleToTorqueAdaptor1(use_a = true, use_w = true)  annotation(
    Placement(visible = true, transformation(origin = {27, 0}, extent = {{-21, -20}, {21, 20}}, rotation = 180)));
  Modelica.Blocks.Math.Gain invertSign(k = -1)  annotation(
    Placement(visible = true, transformation(origin = {62, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput jointAcceleration annotation(
    Placement(visible = true, transformation(origin = {120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Blocks.Nonlinear.FixedDelay fixedDelay1(delayTime = 0.1)  annotation(
    Placement(visible = true, transformation(origin = {-54, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(fixedDelay1.u, actuatorInput) annotation(
    Line(points = {{-66, 0}, {-108, 0}, {-108, 0}, {-120, 0}}, color = {0, 0, 127}));
  connect(fixedDelay1.y, torque1.tau) annotation(
    Line(points = {{-42, 0}, {-22, 0}, {-22, 0}, {-20, 0}}, color = {0, 0, 127}));
  connect(torque1.flange, angleToTorqueAdaptor1.flange) annotation(
    Line(points = {{2, 0}, {23, 0}}));
  connect(jointAcceleration, angleToTorqueAdaptor1.a) annotation(
    Line(points = {{120, 20}, {60, 20}, {60, 6}, {36, 6}, {36, 6}}, color = {0, 0, 127}));
  connect(jointVelocity, angleToTorqueAdaptor1.w) annotation(
    Line(points = {{120, -20}, {76.5, -20}, {76.5, -6}, {35, -6}}, color = {0, 0, 127}));
  connect(jointPosition, angleToTorqueAdaptor1.phi) annotation(
    Line(points = {{120, -60}, {70.5, -60}, {70.5, -16}, {35, -16}}, color = {0, 0, 127}));
  connect(angleToTorqueAdaptor1.tau, invertSign.u) annotation(
    Line(points = {{34, 16}, {42, 16}, {42, 60}, {50, 60}}, color = {0, 0, 127}));
  connect(invertSign.y, jointTorque) annotation(
    Line(points = {{73, 60}, {110, 60}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end DelayTransmission;