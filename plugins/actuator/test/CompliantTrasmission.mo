model CompliantTrasmission
  Modelica.Blocks.Interfaces.RealInput actuatorInput annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput jointPosition annotation(
    Placement(visible = true, transformation(origin = {120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput jointVelocity annotation(
    Placement(visible = true, transformation(origin = {118, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {118, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealOutput jointTorque annotation(
    Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.AngleToTorqueAdaptor angleToTorqueAdaptor1(use_a = false, use_w = true) annotation(
    Placement(visible = true, transformation(origin = {41, 0}, extent = {{-21, -20}, {21, 20}}, rotation = 180)));
  Modelica.Mechanics.Rotational.Sources.Torque torque1 annotation(
    Placement(visible = true, transformation(origin = {-76, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 0.001)  annotation(
    Placement(visible = true, transformation(origin = {-44, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Damper damper1(d = 0.001)  annotation(
    Placement(visible = true, transformation(origin = {-12, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Spring spring1(c = 0.001)  annotation(
    Placement(visible = true, transformation(origin = {16, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(spring1.flange_a, damper1.flange_b) annotation(
    Line(points = {{6, 0}, {-2, 0}}));
  connect(spring1.flange_b, angleToTorqueAdaptor1.flange) annotation(
    Line(points = {{26, 0}, {36, 0}}));
  connect(inertia1.flange_b, damper1.flange_a) annotation(
    Line(points = {{-34, 0}, {-22, 0}}));
  connect(inertia1.flange_a, torque1.flange) annotation(
    Line(points = {{-54, 0}, {-66, 0}}));
  connect(actuatorInput, torque1.tau) annotation(
    Line(points = {{-120, 0}, {-88, 0}}, color = {0, 0, 127}));
  connect(jointPosition, angleToTorqueAdaptor1.phi) annotation(
    Line(points = {{120, -60}, {80.5, -60}, {80.5, -16}, {49, -16}}, color = {0, 0, 127}));
  connect(angleToTorqueAdaptor1.tau, jointTorque) annotation(
    Line(points = {{47, 16}, {80, 16}, {80, 60}, {110, 60}}, color = {0, 0, 127}));
  connect(jointVelocity, angleToTorqueAdaptor1.w) annotation(
    Line(points = {{118, 0}, {79.5, 0}, {79.5, -6}, {49, -6}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end CompliantTrasmission;