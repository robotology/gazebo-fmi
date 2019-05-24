model NullFriction
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
    Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(zero.y, fluidDynamicForce_z) annotation(
    Line(points = {{2, 0}, {70, 0}, {70, 30}, {110, 30}}, color = {0, 0, 127}));
  connect(zero.y, fluidDynamicForce_y) annotation(
    Line(points = {{2, 0}, {50, 0}, {50, 50}, {110, 50}}, color = {0, 0, 127}));
  connect(zero.y, fluidDynamicForce_x) annotation(
    Line(points = {{2, 0}, {30, 0}, {30, 70}, {110, 70}}, color = {0, 0, 127}));
  connect(fluidDynamicMoment_x, zero.y) annotation(
    Line(points = {{110, -30}, {80, -30}, {80, 0}, {2, 0}, {2, 0}, {2, 0}}, color = {0, 0, 127}));
  connect(fluidDynamicMoment_y, zero.y) annotation(
    Line(points = {{110, -50}, {60, -50}, {60, 0}, {0, 0}, {0, 0}, {2, 0}}, color = {0, 0, 127}));
  connect(zero.y, fluidDynamicMoment_z) annotation(
    Line(points = {{1, 0}, {40, 0}, {40, -70}, {110, -70}}, color = {0, 0, 127}));
  annotation(
    uses(Modelica(version = "3.2.2")));
end NullFriction;