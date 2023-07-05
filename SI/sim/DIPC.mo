model DIPC "double inverted pendulum on a cart"
  inner Modelica.Mechanics.MultiBody.World world annotation(Placement(visible = true, transformation(origin = {-115, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic1(useAxisFlange = true) annotation(Placement(visible = true, transformation(origin = {-85, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyBox cart annotation(Placement(visible = true, transformation(origin = {-55, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyShape top_pend(r = {0, top_L, 0}, r_CM = {0, top_lc, 0}, m = top_mass, I_33 = top_inertia, I_22 = 0, I_11 = 0) annotation(Placement(visible = true, transformation(origin = {5, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyShape end_pend(r = {0, end_L, 0}, r_CM = {0, end_lc, 0}, m = end_mass, I_11 = 0, I_22 = 0, I_33 = end_inertia) annotation(Placement(visible = true, transformation(origin = {65, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sources.Accelerate accelerate1 annotation(Placement(visible = true, transformation(origin = {-115, 45}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(useAxisFlange = true, phi.start = 0, w.start = 0) annotation(Placement(visible = true, transformation(origin = {-25, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute2(useAxisFlange = true, phi.start = 0, w.start = 0) annotation(Placement(visible = true, transformation(origin = {35, 5}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Damper damper1(d = top_friction) annotation(Placement(visible = true, transformation(origin = {-25, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Damper damper2(d = end_friction) annotation(Placement(visible = true, transformation(origin = {35, 35}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter Modelica.Units.SI.RotationalDampingConstant top_friction = 0.0001 "friction of cart joint" annotation(Dialog(group = "Variables", showStartAttribute = true));
  parameter Modelica.Units.SI.RotationalDampingConstant end_friction = 0.0001 "friction of mid-pend joint" annotation(Dialog(group = "Variables", showStartAttribute = true));
  parameter Modelica.Units.SI.Mass top_mass = 0.1 "mass of first pendulum";
  parameter Modelica.Units.SI.Mass end_mass = 0.1 "mass of end pendulum";
  parameter Modelica.Units.SI.Length top_L = 0.3 "length of top pendulum";
  parameter Modelica.Units.SI.Length top_lc = 0.15 "distance from cart pivot to first pendulum COM";
  parameter Modelica.Units.SI.Length end_lc = 0.15 "distance from mid pivot to end pendulum COM";
  parameter Modelica.Units.SI.MomentOfInertia top_inertia = 0.0001 "top pendulum rotational inertia about z axis";
  parameter Modelica.Units.SI.MomentOfInertia end_inertia = 0.0001 "end pendulum rotational inertia about z axis";
  parameter Modelica.Units.SI.Length end_L = 0.3 "length of end pendulum";
  Modelica.Blocks.Interfaces.RealInput u annotation(Placement(visible = true, transformation(origin = {-155, 45}, extent = {{-11.425, -11.425}, {11.425, 11.425}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.RelSpeedSensor relSpeedSensor1 annotation(Placement(visible = true, transformation(origin = {-25, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.RelAngleSensor relAngleSensor1 annotation(Placement(visible = true, transformation(origin = {-25, -75}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.RelAngleSensor relAngleSensor2 annotation(Placement(visible = true, transformation(origin = {35, -75}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.RelSpeedSensor relSpeedSensor2 annotation(Placement(visible = true, transformation(origin = {35, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sensors.RelSpeedSensor relSpeedSensor3 annotation(Placement(visible = true, transformation(origin = {-85, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sensors.RelPositionSensor relPositionSensor1 annotation(Placement(visible = true, transformation(origin = {-85, -75}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y0 annotation(Placement(visible = true, transformation(origin = {-85, -55}, extent = {{-5.784, -5.784}, {5.784, 5.784}}, rotation = -90), iconTransformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y1 annotation(Placement(visible = true, transformation(origin = {-25, -55}, extent = {{-5.784, -5.784}, {5.784, 5.784}}, rotation = -90), iconTransformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y2 annotation(Placement(visible = true, transformation(origin = {35, -55}, extent = {{-5.784, -5.784}, {5.784, 5.784}}, rotation = -90), iconTransformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y3 annotation(Placement(visible = true, transformation(origin = {-85, -99.216}, extent = {{-5.784, -5.784}, {5.784, 5.784}}, rotation = -90), iconTransformation(origin = {100, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y4 annotation(Placement(visible = true, transformation(origin = {-25, -99.216}, extent = {{-5.784, -5.784}, {5.784, 5.784}}, rotation = -90), iconTransformation(origin = {100, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput y5 annotation(Placement(visible = true, transformation(origin = {35, -99.216}, extent = {{-5.784, -5.784}, {5.784, 5.784}}, rotation = -90), iconTransformation(origin = {100, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(prismatic1.frame_b, cart.frame_a) annotation(Line(visible = true, origin = {-70, 5}, points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}));
  connect(world.frame_b, prismatic1.frame_a) annotation(Line(visible = true, origin = {-100, 5}, points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}));
  connect(accelerate1.flange, prismatic1.axis) annotation(Line(visible = true, origin = {-84.011, 28.984}, points = {{-20.989, 16.016}, {9.011, 16.016}, {9.011, -16.016}, {7.011, -17.984}}, color = {0, 127, 0}));
  connect(cart.frame_b, revolute1.frame_a) annotation(Line(visible = true, origin = {-40, 5}, points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}));
  connect(revolute1.frame_b, top_pend.frame_a) annotation(Line(visible = true, origin = {-10, 5}, points = {{-5, 0}, {5, 0}}, color = {95, 95, 95}));
  connect(revolute2.frame_a, top_pend.frame_b) annotation(Line(visible = true, origin = {20, 5}, points = {{5, 0}, {-5, 0}}, color = {95, 95, 95}));
  connect(revolute2.frame_b, end_pend.frame_a) annotation(Line(visible = true, origin = {50, 5}, points = {{-5, 0}, {5, -0}}, color = {95, 95, 95}));
  connect(damper1.flange_b, revolute1.axis) annotation(Line(visible = true, origin = {-15.5, 22}, points = {{0.5, 13}, {5.5, 13}, {5.5, -4.5}, {-9.5, -4.5}, {-9.5, -7}}, color = {64, 64, 64}));
  connect(revolute1.support, damper1.flange_a) annotation(Line(visible = true, origin = {-36.9, 22}, points = {{5.9, -7}, {5.9, -4.5}, {-3.1, -4.5}, {-3.1, 13}, {1.9, 13}}, color = {64, 64, 64}));
  connect(damper2.flange_a, revolute2.support) annotation(Line(visible = true, origin = {24.6, 25}, points = {{0.4, 10}, {-4.6, 10}, {-4.6, -5}, {4.4, -5}, {4.4, -10}}, color = {64, 64, 64}));
  connect(revolute2.axis, damper2.flange_b) annotation(Line(visible = true, origin = {43, 25}, points = {{-8, -10}, {-8, -5}, {7, -5}, {7, 10}, {2, 10}}, color = {64, 64, 64}));
  connect(u, accelerate1.a_ref) annotation(Line(points = {{-155, 45}, {-127, 45}}, color = {1, 37, 163}));
  connect(relSpeedSensor1.flange_a, damper1.flange_a) annotation(Line(visible = true, origin = {-37.5, 2.5}, points = {{2.5, -32.5}, {-2.5, -32.5}, {-2.5, 32.5}, {2.5, 32.5}}, color = {64, 64, 64}));
  connect(relSpeedSensor1.flange_b, damper1.flange_b) annotation(Line(visible = true, origin = {-12.5, 2.5}, points = {{-2.5, -32.5}, {2.5, -32.5}, {2.5, 32.5}, {-2.5, 32.5}}, color = {64, 64, 64}));
  connect(relAngleSensor1.flange_a, relSpeedSensor1.flange_a) annotation(Line(visible = true, origin = {-37.5, -52.5}, points = {{2.5, -22.5}, {-2.5, -22.5}, {-2.5, 22.5}, {2.5, 22.5}}, color = {64, 64, 64}));
  connect(relAngleSensor1.flange_b, relSpeedSensor1.flange_b) annotation(Line(visible = true, origin = {-12.5, -52.5}, points = {{-2.5, -22.5}, {2.5, -22.5}, {2.5, 22.5}, {-2.5, 22.5}}, color = {64, 64, 64}));
  connect(relSpeedSensor2.flange_b, revolute2.axis) annotation(Line(visible = true, origin = {43, -1}, points = {{2, -29}, {7, -29}, {7, 21}, {-8, 21}, {-8, 16}}, color = {64, 64, 64}));
  connect(relSpeedSensor2.flange_a, revolute2.support) annotation(Line(visible = true, origin = {24.6, -1}, points = {{0.4, -29}, {-4.6, -29}, {-4.6, 21}, {4.4, 21}, {4.4, 16}}, color = {64, 64, 64}));
  connect(relAngleSensor2.flange_a, revolute2.support) annotation(Line(visible = true, origin = {24.6, -19}, points = {{0.4, -56}, {-4.6, -56}, {-4.6, 39}, {4.4, 39}, {4.4, 34}}, color = {64, 64, 64}));
  connect(relAngleSensor2.flange_b, revolute2.axis) annotation(Line(visible = true, origin = {43, -19}, points = {{2, -56}, {7, -56}, {7, 39}, {-8, 39}, {-8, 34}}, color = {64, 64, 64}));
  connect(relSpeedSensor3.flange_b, prismatic1.axis) annotation(Line(visible = true, origin = {-73, -9.5}, points = {{-2, -20.5}, {3, -20.5}, {3, 20.5}, {-4, 20.5}}, color = {0, 127, 0}));
  connect(relSpeedSensor3.flange_a, prismatic1.support) annotation(Line(visible = true, origin = {-94.6, -1.8}, points = {{-0.4, -28.2}, {-5.4, -28.2}, {-5.4, 21.8}, {5.6, 21.8}, {5.6, 12.8}}, color = {0, 127, 0}));
  connect(relPositionSensor1.flange_b, prismatic1.axis) annotation(Line(visible = true, origin = {-73, -32}, points = {{-2, -43}, {3, -43}, {3, 43}, {-4, 43}}, color = {0, 127, 0}));
  connect(relPositionSensor1.flange_a, prismatic1.support) annotation(Line(visible = true, origin = {-94.6, -19.8}, points = {{-0.4, -55.2}, {-5.4, -55.2}, {-5.4, 39.8}, {5.6, 39.8}, {5.6, 30.8}}, color = {0, 127, 0}));
  connect(relSpeedSensor3.v_rel, y0) annotation(Line(visible = true, origin = {-85, -48}, points = {{0, 7}, {0, -7}}, color = {1, 37, 163}));
  connect(relSpeedSensor1.w_rel, y1) annotation(Line(visible = true, origin = {-25, -48}, points = {{0, 7}, {-0, -7}}, color = {1, 37, 163}));
  connect(relSpeedSensor2.w_rel, y2) annotation(Line(visible = true, origin = {35, -48}, points = {{0, 7}, {0, -7}}, color = {1, 37, 163}));
  connect(relPositionSensor1.s_rel, y3) annotation(Line(visible = true, origin = {-85, -92.608}, points = {{0, 6.608}, {0, -6.608}}, color = {1, 37, 163}));
  connect(relAngleSensor1.phi_rel, y4) annotation(Line(visible = true, origin = {-25, -92.608}, points = {{0, 6.608}, {0, -6.608}}, color = {1, 37, 163}));
  connect(relAngleSensor2.phi_rel, y5) annotation(Line(visible = true, origin = {35, -92.608}, points = {{0, 6.608}, {0, -6.608}}, color = {1, 37, 163}));
  annotation(
    uses(Modelica(version = "4.0.0"), IntroductoryExamples(version = "13.3")),
    experiment(StopTime = 10.0, __Wolfram_Algorithm = "dassl"),
    Diagram(coordinateSystem(extent = {{-160, -155}, {150, 90}}, preserveAspectRatio = true, initialScale = 0.1, grid = {5, 5})),
    Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {10, 10}), graphics = {Rectangle(lineColor = {0, 114, 195}, fillColor = {255, 255, 255}, extent = {{-100, -100}, {100, 100}}, radius = 25), Text(textColor = {64, 64, 64}, extent = {{-150, 110}, {150, 150}}, textString = "%name")}));
end DIPC;
