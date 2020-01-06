// robot config
let robot_radius = 10.0;

let x_in = 50.0;
let y_in = 50.0;
let theta_in = 0.0;

let scan_range = [radians(-90.0), radians(90.0)];
let scan_offset = radians(10.0);
let scan_dist = 100.0;

// dwa config
let max_lin_vel = 25.0;
let min_lin_vel = 0.0;

let max_ang_vel = 10.0;
let min_ang_vel = -10.0;

let limit_lin_acc = 50.0;
let limit_ang_acc = 5.0;

let vx_samples = 10;
let vth_samples = 10;

let dt = 0.050; // sec
let sim_time = 3.0; //sec

let heading_bias = 1.0;
let velocity_bias = 2.0;
let clearance_bias = 2.0;

// initialization
let axis;
let robot;
let obstacles = [];
let dwa;

let mouse_pressed_cnt = 0;
let t = 0;

let vel = [0.0, 0.0];
let acc = [limit_lin_acc, limit_ang_acc];

let predicted_robot_state = [x_in, y_in, theta_in, vel[0], vel[1]];
let goal_pose = [192.0, 64.0, 216.0, 63.0, -0.04164257909858842];
// let goal_pose = [267.0, 323.0, 268.0, 346.0, 1.5273454314033659];
let resulting_search_space = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

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
    vx_samples,
    vth_samples,
    sim_time,
    dt,
    heading_bias,
    velocity_bias,
    clearance_bias);

  obstacles[0] = new Obstacle(55, 160, 116, 141, 120, 210, 49, 204);
  obstacles[1] = new Obstacle(155, 274, 253, 279, 216, 360, 144, 360);
  obstacles[2] = new Obstacle(264, 40, 347, 45, 358, 136, 264, 40);
  obstacles[3] = new Obstacle(215, 114, 265, 114, 265, 164, 215, 164);
  obstacles[4] = new Obstacle(134, 32, 188, 32, 158, 88, 134, 32);

  // noLoop();
}

function draw()
{
  if (millis() - t > dt * 1000)
  {
    angleMode(RADIANS);
    background(255);
    axis.show(width, height);

    stroke(0);
    strokeWeight(1);
    ellipse(goal_pose[0], goal_pose[1], 5, 5);
    line(goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3]);

    for (let i = 0; i < obstacles.length; i++)
    {
      obstacles[i].show();
    }

    robot.scan_update(obstacles);

    predicted_robot_state = dwa.predict_motion(predicted_robot_state, vel, dt);
    resulting_search_space = dwa.update_search_space(predicted_robot_state);
    vel = dwa.maximizing_objective_function(
      predicted_robot_state,
      resulting_search_space,
      [goal_pose[0], goal_pose[1], goal_pose[4]],
      robot.radius,
      obstacles,
      robot.scan_range,
      robot.scan_offset,
      robot.scan_dist);

    robot.odom_update(vel[0], vel[1], acc[0], acc[1], dt);
    robot.draw();

    let dx = goal_pose[0] - robot.x;
    let dy = goal_pose[1] - robot.y;
    let dth = normalize_angle(goal_pose[4] - robot.theta);
    let dist_error = sqrt(pow(dx, 2) + pow(dy, 2));

    if (dist_error <= robot.radius)
    {
      if (abs(dth) < radians(10))
      {
        console.log("GOAL!!!");
        noLoop();
      }
      else
      {
        if (dth < 0.0)
        {
          vel[1] = -0.3;
        }
        else if (dth > 0.0)
        {
          vel[1] = 0.3;
        }
      }
    }

    debug_code();

    t = millis();
  }
}

function debug_code()
{
  noStroke();
  textSize(8);
  text('lin_vel : ' + robot.lin_vel, width - 60, height - 150);
  text('ang_vel : ' + robot.ang_vel, width - 60, height - 140);
  text('lin_acc : ' + robot.lin_acc, width - 60, height - 130);
  text('ang_acc : ' + robot.ang_acc, width - 60, height - 120);

  text(' ', width - 60, height - 110);
  text('[current]', width - 60, height - 100);
  text('x : ' + robot.x, width - 60, height - 90);
  text('y : ' + robot.y, width - 60, height - 80);
  text('theta : ' + robot.theta, width - 60, height - 70);

  text(' ', width - 60, height - 60);
  text('[target]', width - 60, height - 50);
  text('x : ' + goal_pose[0], width - 60, height - 40);
  text('y : ' + goal_pose[1], width - 60, height - 30);
  text('theta : ' + goal_pose[4], width - 60, height - 20);
}

function keyPressed()
{
  let lin_vel = vel[0];
  let ang_vel = vel[1];

  acc[0] = limit_lin_acc;
  acc[1] = limit_ang_acc;

  let lin_vel_step = 3.0;
  let ang_vel_step = 1.0;

  if (key == 'w')
  {
    lin_vel = lin_vel + lin_vel_step;
    if (lin_vel >= max_lin_vel)
    {
      lin_vel = max_lin_vel;
    }
  }
  else if (key == 'x')
  {
    lin_vel = lin_vel - lin_vel_step;
    if (lin_vel <= min_lin_vel)
    {
      lin_vel = min_lin_vel;
    }
  }
  else if (key == 'a')
  {
    ang_vel = ang_vel - ang_vel_step;
    if (ang_vel <= min_ang_vel)
    {
      ang_vel = min_ang_vel;
    }
  }
  else if (key == 'd')
  {
    ang_vel = ang_vel + ang_vel_step;
    if (ang_vel >= max_ang_vel)
    {
      ang_vel = max_ang_vel;
    }
  }
  else if (key == 's')
  {
    lin_vel = 0.0;
    ang_vel = 0.0;
  }

  vel[0] = lin_vel;
  vel[1] = ang_vel;
}

function mousePressed()
{
  if (mouse_pressed_cnt == 0)
  {
    noLoop();
    goal_pose = [];
    goal_pose.push(mouseX);
    goal_pose.push(mouseY);
    mouse_pressed_cnt++;
  }
  else if (mouse_pressed_cnt == 1)
  {
    goal_pose.push(mouseX);
    goal_pose.push(mouseY);
    goal_pose.push(atan2(goal_pose[3] - goal_pose[1], goal_pose[2] - goal_pose[0]));

    // re-initialization
    robot = new Robot(robot_radius, x_in, y_in, theta_in, scan_range, scan_offset, scan_dist);
    dwa = new DWA(
      max_lin_vel,
      min_lin_vel,
      max_ang_vel,
      min_ang_vel,
      limit_lin_acc,
      limit_ang_acc,
      vx_samples,
      vth_samples,
      sim_time,
      dt,
      heading_bias,
      velocity_bias,
      clearance_bias);

    vel = [0.0, 0.0];
    acc = [limit_lin_acc, limit_ang_acc];

    predicted_robot_state = [x_in, y_in, theta_in, vel[0], vel[1]];

    mouse_pressed_cnt = 0;

    loop();
  }
}
