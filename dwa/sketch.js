class DWA
{
  constructor(
    robot_radius,
    max_vel,
    min_vel,
    max_yawrate,
    max_accel,
    max_dyawrate,
    v_reso,
    dt,
    predict_time,
    heading_cost_gain,
    velocity_cost_gain,
    clearance_cost_gain)
  {
    this.max_vel = max_vel;
    this.min_vel = min_vel;
    this.max_yawrate = max_yawrate;
    this.max_accel = max_accel;

    this.max_dyawrate = max_dyawrate;

    this.v_reso = v_reso;
    this.dt = dt;
    this.predict_time = predict_time;

    this.heading_cost_gain = heading_cost_gain;
    this.velocity_cost_gain = velocity_cost_gain;
    this.clearance_cost_gain = clearance_cost_gain;

    this.robot_radius = robot_radius;
  }

  motion_predict()
  {

  }

  heading()
  {

  }

  clearance()
  {

  }

  velocity()
  {

  }
}

// robot config
let robot_radius = 10.0;

let x_in = 50.0;
let y_in = 50.0;
let theta_in = 0.0;

let scan_range = [radians(-90.0), radians(90.0)];
let scan_offset = radians(10.0);
let scan_dist = 100.0;

var axis;
var robot;
var obstacles = [];
var dwa;

var t = 0;
var interval = 50;

var lin_vel = 0.0;
var ang_vel = 0.0;
var lin_acc = 0.0;
var ang_acc = 0.0;

var goal_pose = {x: 0.0, y: 0.0};

function setup()
{
  createCanvas(400, 400);

  axis = new Axis();
  robot = new Robot(robot_radius, x_in, y_in, theta_in, scan_range, scan_offset, scan_dist);

  obstacles[0] = new Obstacle(55, 160, 116, 141, 120, 210, 49, 204);
  obstacles[1] = new Obstacle(155, 274, 253, 279, 216, 360, 144, 360);
  obstacles[2] = new Obstacle(264, 40, 347, 45, 358, 136, 264, 40);
  obstacles[3] = new Obstacle(215, 114, 265, 114, 265, 164, 215, 164);
  obstacles[4] = new Obstacle(134, 32, 188, 32, 158, 88, 134, 32);

  noLoop();
}

function draw()
{
  if (millis() - t > interval)
  {
    axis.show(width, height);
    ellipse(goal_pose.x, goal_pose.y, 5, 5);

    for (var i = 0; i < obstacles.length; i++)
    {
      obstacles[i].show();
    }

    robot.odom_update(lin_vel, ang_vel, lin_acc, ang_acc, interval / 1000);
    robot.scan_update(obstacles);
    robot.draw();

    noStroke();
    textSize(8);
    text('x : ' + robot.x, width - 80, height - 40);
    text('y : ' + robot.y, width - 80, height - 30);
    text('theta : ' + robot.theta, width - 80, height - 20);

    text('lin_vel : ' + robot.lin_vel, width - 80, height - 60);
    text('ang_vel : ' + robot.ang_vel, width - 80, height - 50);

    t = millis();
  }
}

function keyPressed()
{
  lin_acc = 1.0;
  ang_acc = 0.05;

  if (key == 'w')
  {
    lin_vel = lin_vel + 5.0;
  }
  else if (key == 'x')
  {
    lin_vel = lin_vel - 5.0;
  }
  else if (key == 'a')
  {
    ang_vel = ang_vel - 0.5;
  }
  else if (key == 'd')
  {
    ang_vel = ang_vel + 0.5;
  }
  else if (key == 's')
  {
    lin_vel = 0.0;
    ang_vel = 0.0;
  }
}

function mousePressed()
{
  noLoop();

  goal_pose = {x: mouseX, y: mouseY};
  robot = new Robot(robot_radius, x_in, y_in, theta_in, scan_range, scan_offset, scan_dist);
  lin_vel = 0.0;
  ang_vel = 0.0;

  loop();
}
