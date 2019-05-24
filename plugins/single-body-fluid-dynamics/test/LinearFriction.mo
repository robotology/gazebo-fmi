model LinearFriction
  Modelica.Blocks.Interfaces.RealInput relativeVelocity_x annotation(
    Placement(visible = true, transformation(origin = {-120, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput relativeVelocity_y annotation(
    Placement(visible = true, transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput relativeVelocity_z annotation(
    Placement(visible = true, transformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {-120, -50}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput fluidDynamicForce_x annotation(
    Placement(visible = true, transformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput fluidDynamicForce_y annotation(
    Placement(visible = true, transformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput fluidDynamicForce_z annotation(
    Placement(visible = true, transformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput fluidDynamicMoment_x annotation(
    Placement(visible = true, transformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput fluidDynamicMoment_y annotation(
    Placement(visible = true, transformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput fluidDynamicMoment_z annotation(
    Placement(visible = true, transformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {110, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant zero(k = 0)  annotation(
    Placement(visible = true, transformation(origin = {50, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain_x(k = -10)  annotation(
    Placement(visible = true, transformation(origin = {10, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain_y(k = -10)  annotation(
    Placement(visible = true, transformation(origin = {40, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain_z(k = -10)  annotation(
    Placement(visible = true, transformation(origin = {70, 30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(relativeVelocity_z, gain_z.u) annotation(
    Line(points = {{-120, -50}, {20, -50}, {20, 30}, {58, 30}, {58, 30}}, color = {0, 0, 127}));
  connect(gain_x.y, fluidDynamicForce_x) annotation(
    Line(points = {{22, 70}, {104, 70}, {104, 70}, {110, 70}}, color = {0, 0, 127}));
  connect(gain_y.y, fluidDynamicForce_y) annotation(
    Line(points = {{52, 50}, {102, 50}, {102, 50}, {110, 50}}, color = {0, 0, 127}));
  connect(gain_z.y, fluidDynamicForce_z) annotation(
    Line(points = {{82, 30}, {102, 30}, {102, 30}, {110, 30}}, color = {0, 0, 127}));
  connect(relativeVelocity_y, gain_y.u) annotation(
    Line(points = {{-120, 0}, {0, 0}, {0, 50}, {28, 50}, {28, 50}, {28, 50}}, color = {0, 0, 127}));
  connect(relativeVelocity_x, gain_x.u) annotation(
    Line(points = {{-120, 50}, {-20, 50}, {-20, 70}, {-4, 70}, {-4, 70}, {-2, 70}}, color = {0, 0, 127}));
  connect(zero.y, fluidDynamicMoment_z) annotation(
    Line(points = {{62, -50}, {80, -50}, {80, -70}, {110, -70}, {110, -70}}, color = {0, 0, 127}));
  connect(zero.y, fluidDynamicMoment_x) annotation(
    Line(points = {{62, -50}, {80, -50}, {80, -30}, {110, -30}, {110, -30}}, color = {0, 0, 127}));
  connect(zero.y, fluidDynamicMoment_y) annotation(
    Line(points = {{62, -50}, {110, -50}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end LinearFriction;