class DWA
{
  constructor(
    max_lin_vel,
    min_lin_vel,
    max_ang_vel,
    min_ang_vel,
    limit_lin_acc,
    limit_ang_acc,
    v_reso,
    yawrate_reso,
    dt,
    predict_time,
    heading_cost_gain,
    velocity_cost_gain,
    clearance_cost_gain)
  {
    this.max_lin_vel = max_lin_vel;
    this.min_lin_vel = min_lin_vel;

    this.max_ang_vel = max_ang_vel;
    this.min_ang_vel = min_ang_vel;

    this.limit_lin_acc = limit_lin_acc;
    this.limit_ang_acc = limit_ang_acc;

    this.v_reso = v_reso;
    this.yawrate_reso = yawrate_reso;

    this.dt = dt;
    this.predict_time = predict_time;

    this.heading_cost_gain = heading_cost_gain;
    this.velocity_cost_gain = velocity_cost_gain;
    this.clearance_cost_gain = clearance_cost_gain;
  }

  motion_predict(state, input, dt)
  {
    // state[] : x_pos, y_pos, theta, lin_vel, ang_vel
    // input[] : lin_vel, ang_vel
    // dt : control period time

    state[2] += input[1] * dt;

    state[0] += input[0] * dt * cos(state[2]);
    state[1] += input[0] * dt * sin(state[2]);

    state[3] = input[0];
    state[4] = input[1];

    return state;
  }

  update_dynamic_window(state, scan_data)
  {
    // update possible velocity
    let Vs = [this.min_lin_vel, this.max_lin_vel, this.min_ang_vel, this.max_ang_vel];

    // update admissible velocity
    let Va = [];
    for (let i = 0; i < scan_data.length; i++)
    {
      Va.push([
        this.min_lin_vel,
        sqrt(2 * scan_data[i] * this.limit_lin_acc),
        this.min_ang_vel,
        sqrt(2 * scan_data[i] * this.limit_ang_acc)]);
    }

    // update dynamic window velocity
    let Vd = [
      state[3] - this.limit_lin_acc * this.dt,
      state[3] + this.limit_lin_acc * this.dt,
      state[4] - this.limit_ang_acc * this.dt,
      state[4] + this.limit_ang_acc * this.dt];

    // update resulting velocity
    let Vr = [];
    for (let i = 0; i < scan_data.length; i++)
    {
      Vr.push([
      max(Vd[0], max(Vs[0], Va[i][0])),
      min(Vd[1], min(Vs[1], Va[i][1])),
      max(Vd[2], max(Vs[2], Va[i][2])),
      max(Vd[3], max(Vs[3], Va[i][3]))]);
    }

    return Vr;
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

// dwa config
let max_lin_vel = 5.0;
let min_lin_vel = 0.0;

let max_ang_vel = 0.5;
let min_ang_vel = -0.5;

let limit_lin_acc = 1.0;
let limit_ang_acc = 0.025;

let v_reso = 0.01;
let yawrate_reso = 0.1;

let dt = 0.050; // sec
let predict_time = 3.0; //sec

let heading_cost_gain = 1.0;
let velocity_cost_gain = 1.0;
let clearance_cost_gain = 1.0;

// initialization
let axis;
let robot;
let obstacles = [];
let dwa;

let t = 0;

let lin_vel = 0.0;
let ang_vel = 0.0;
let lin_acc = 0.0;
let ang_acc = 0.0;

let robot_state = [x_in, y_in, theta_in, lin_vel, ang_vel];
let goal_pose = {x: 0.0, y: 0.0};
let result = [];

function setup()
{
  createCanvas(400, 400);

  axis = new Axis();
  robot = new Robot(robot_radius, x_in, y_in, theta_in, scan_range, scan_offset, scan_dist);
  dwa = new DWA(
    max_lin_vel,
    min_lin_vel,
    max_ang_vel,
    min_ang_vel,
    limit_lin_acc,
    limit_ang_acc,
    v_reso,
    yawrate_reso,
    dt,
    predict_time,
    heading_cost_gain,
    velocity_cost_gain,
    clearance_cost_gain);

  obstacles[0] = new Obstacle(55, 160, 116, 141, 120, 210, 49, 204);
  obstacles[1] = new Obstacle(155, 274, 253, 279, 216, 360, 144, 360);
  obstacles[2] = new Obstacle(264, 40, 347, 45, 358, 136, 264, 40);
  obstacles[3] = new Obstacle(215, 114, 265, 114, 265, 164, 215, 164);
  obstacles[4] = new Obstacle(134, 32, 188, 32, 158, 88, 134, 32);

  noLoop();
}

function draw()
{
  if (millis() - t > dt * 1000)
  {
    axis.show(width, height);
    ellipse(goal_pose.x, goal_pose.y, 5, 5);

    for (let i = 0; i < obstacles.length; i++)
    {
      obstacles[i].show();
    }


    robot.odom_update(lin_vel, ang_vel, lin_acc, ang_acc, dt);
    robot.scan_update(obstacles);

    robot_state = dwa.motion_predict(robot_state, [lin_vel, ang_vel], dt);
    result = dwa.update_dynamic_window(robot_state, robot.scan_data);
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
  lin_acc = limit_lin_acc;
  ang_acc = limit_ang_acc;

  if (key == 'w')
  {
    lin_vel = lin_vel + max_lin_vel;
  }
  else if (key == 'x')
  {
    lin_vel = lin_vel - max_lin_vel;
  }
  else if (key == 'a')
  {
    ang_vel = ang_vel - max_ang_vel;
  }
  else if (key == 'd')
  {
    ang_vel = ang_vel + max_ang_vel;
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

  robot_state = [x_in, y_in, theta_in, lin_vel, ang_vel];

  loop();
}
