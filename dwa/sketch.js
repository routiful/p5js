// robot config
let robot_radius = 10.0;

let x_in = 50.0;
let y_in = 50.0;
let theta_in = 0.0;

let scan_range = [radians(-90.0), radians(90.0)];
let scan_offset = radians(10.0);
let scan_dist = 100.0;

// dwa config
let max_lin_vel = 10.0;
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

let mouse_pressed_cnt = 0;
let t = 0;

let vel = [0.0, 0.0];
let acc = [limit_lin_acc, limit_ang_acc];

let predicted_robot_state = [x_in, y_in, theta_in, vel[0], vel[1]];
let goal_pose = [];
let resulting_search_space = [];

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

    stroke(0);
    strokeWeight(1);
    ellipse(goal_pose[0], goal_pose[1], 5, 5);
    line(goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3]);

    for (let i = 0; i < obstacles.length; i++)
    {
      obstacles[i].show();
    }

    robot.scan_update(obstacles);

    predicted_robot_state = dwa.motion_predict(predicted_robot_state, vel, dt);
    resulting_search_space = dwa.update_search_space(predicted_robot_state, min(robot.scan_data), acc);
    // vel = dwa.maximizing_objective_function(
    //   predicted_robot_state,
    //   dynamic_window,
    //   [goal_pose[0], goal_pose[1], goal_pose[4]],
    //   robot.scan_data);

    robot.odom_update(vel[0], vel[1], acc[0], acc[1], dt);
    robot.draw();

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

  let lin_vel_step = 2.5;
  let ang_vel_step = 0.025;

  if (key == 'w')
  {
    lin_vel = lin_vel + lin_vel_step;
  }
  else if (key == 'x')
  {
    lin_vel = lin_vel - lin_vel_step;
  }
  else if (key == 'a')
  {
    ang_vel = ang_vel - ang_vel_step;
  }
  else if (key == 'd')
  {
    ang_vel = ang_vel + ang_vel_step;
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

    vel = [0.0, 0.0];
    acc = [0.0, 0.0];

    predicted_robot_state = [x_in, y_in, theta_in, vel[0], vel[1]];

    mouse_pressed_cnt = 0;

    loop();
  }
}
