model NullTrasmission
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
    Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant ConstZero(k = 0)  annotation(
    Placement(visible = true, transformation(origin = {-58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(ConstZero.y, torque1.tau) annotation(
    Line(points = {{-47, 0}, {-22, 0}}, color = {0, 0, 127}));
  connect(torque1.flange, angleToTorqueAdaptor1.flange) annotation(
    Line(points = {{0, 0}, {37, 0}}));
  connect(jointPosition, angleToTorqueAdaptor1.phi) annotation(
    Line(points = {{120, -60}, {80.5, -60}, {80.5, -16}, {49, -16}}, color = {0, 0, 127}));
  connect(angleToTorqueAdaptor1.tau, jointTorque) annotation(
    Line(points = {{47, 16}, {80, 16}, {80, 60}, {110, 60}}, color = {0, 0, 127}));
  connect(jointVelocity, angleToTorqueAdaptor1.w) annotation(
    Line(points = {{118, 0}, {79.5, 0}, {79.5, -6}, {49, -6}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end NullTrasmission;