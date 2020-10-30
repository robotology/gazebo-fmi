model SimpleElectricalActuator
  Modelica.Blocks.Interfaces.RealInput actuatorInput annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput jointPosition annotation(
    Placement(visible = true, transformation(origin = {120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {120, -60}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealInput jointVelocity annotation(
    Placement(visible = true, transformation(origin = {120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {120, -20}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Blocks.Interfaces.RealOutput jointTorque annotation(
    Placement(visible = true, transformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.AngleToTorqueAdaptor angleToTorqueAdaptor1(use_a = true, use_w = true) annotation(
    Placement(visible = true, transformation(origin = {63, 0}, extent = {{-21, -20}, {21, 20}}, rotation = 180)));
  Modelica.Blocks.Math.Gain invertSign(k = -1)  annotation(
    Placement(visible = true, transformation(origin = {86, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput jointAcceleration annotation(
    Placement(visible = true, transformation(origin = {120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 180), iconTransformation(origin = {120, 20}, extent = {{-20, -20}, {20, 20}}, rotation = 180)));
  Modelica.Mechanics.Rotational.Components.IdealGear idealGear1(ratio = 100)  annotation(
    Placement(visible = true, transformation(origin = {24, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Damper damper1(d = 0.25)  annotation(
    Placement(visible = true, transformation(origin = {44, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Electrical.Analog.Sources.SignalVoltage Vs
    annotation (Placement(visible = true, transformation(origin = {-54, -2.66454e-15}, extent = {{-16, 16}, {16, -16}}, rotation = 270)));
  Modelica.Electrical.Analog.Basic.Ground ground1 annotation(
    Placement(visible = true, transformation(origin = {-40, -56}, extent = {{-14, -14}, {14, 14}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.EMF emf(k = 1e-3)  annotation(
    Placement(visible = true, transformation(origin = {-23, 1}, extent = {{-13, -13}, {13, 13}}, rotation = 0)));
  Modelica.Electrical.Analog.Basic.Resistor resistor1(R = 1)  annotation(
    Placement(visible = true, transformation(origin = {-38, 56}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Inertia inertia1(J = 1e-5)  annotation(
    Placement(visible = true, transformation(origin = {8, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter1(limitsAtInit = true, uMax = 48, uMin = -48)  annotation(
    Placement(visible = true, transformation(origin = {-84, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(inertia1.flange_b, idealGear1.flange_a) annotation(
    Line(points = {{18, 0}, {18, 32}, {14, 32}, {14, 52}}));
  connect(emf.flange, inertia1.flange_a) annotation(
    Line(points = {{-10, 1}, {-10, 7.5}, {-2, 7.5}, {-2, 0}}));
  connect(damper1.flange_a, idealGear1.flange_b) annotation(
    Line(points = {{34, 0}, {34, 52}}));
  connect(resistor1.n, emf.p) annotation(
    Line(points = {{-22, 56}, {-22, 25}, {-23, 25}, {-23, 14}}, color = {0, 0, 255}));
  connect(emf.n, ground1.p) annotation(
    Line(points = {{-23, -12}, {-24, -12}, {-24, -42}, {-40, -42}}, color = {0, 0, 255}));
  connect(limiter1.y, Vs.v) annotation(
    Line(points = {{-73, 0}, {-65, 0}}, color = {0, 0, 127}));
  connect(Vs.p, resistor1.p) annotation(
    Line(points = {{-54, 16}, {-54, 56}}, color = {0, 0, 255}));
  connect(Vs.n, ground1.p) annotation(
    Line(points = {{-54, -16}, {-54, -42}, {-40, -42}}, color = {0, 0, 255}));
  connect(actuatorInput, limiter1.u) annotation(
    Line(points = {{-120, 0}, {-96, 0}}, color = {0, 0, 127}));
  connect(damper1.flange_b, angleToTorqueAdaptor1.flange) annotation(
    Line(points = {{54, 0}, {59, 0}}));
  connect(jointPosition, angleToTorqueAdaptor1.phi) annotation(
    Line(points = {{120, -60}, {80.5, -60}, {80.5, -16}, {71, -16}}, color = {0, 0, 127}));
  connect(jointVelocity, angleToTorqueAdaptor1.w) annotation(
    Line(points = {{120, -20}, {83.5, -20}, {83.5, -6}, {71, -6}}, color = {0, 0, 127}));
  connect(angleToTorqueAdaptor1.tau, invertSign.u) annotation(
    Line(points = {{69, 16}, {69, 60}, {74, 60}}, color = {0, 0, 127}));
  connect(jointAcceleration, angleToTorqueAdaptor1.a) annotation(
    Line(points = {{120, 20}, {80, 20}, {80, 6}, {71, 6}}, color = {0, 0, 127}));
  connect(invertSign.y, jointTorque) annotation(
    Line(points = {{97, 60}, {110, 60}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end SimpleElectricalActuator;