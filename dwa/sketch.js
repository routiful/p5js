class DWA
{
  constructor(max_tv, min_tv, max_rv, min_rv)
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

var axis = new Axis();
var robot = new Robot(robot_radius, x_in, y_in, theta_in, scan_range, scan_offset, scan_dist);
var obstacles = [];

var t = 0;
var interval = 50;

var lin_vel = 0.0;
var ang_vel = 0.0;

var goal_pose = {x: 0.0, y: 0.0};

function setup()
{
  createCanvas(400, 400);

  // obstacles[0] = new Obstacle(100, 100, 100, 150, 150, 150, 150, 100);
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

    robot.odom_update(lin_vel, ang_vel, interval / 1000);
    robot.scan_update(obstacles);
    robot.draw();

    noStroke();
    textSize(8);
    text('x : ' + robot.x, width - 80, height - 40);
    text('y : ' + robot.y, width - 80, height - 30);
    text('theta : ' + robot.theta, width - 80, height - 20);

    text('lin_vel : ' + lin_vel, width - 80, height - 60);
    text('ang_vel : ' + ang_vel, width - 80, height - 50);

    t = millis();
  }
}

function keyPressed()
{
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

function Axis()
{
  this.show = function(x, y)
  {
    background(255);

    for (let i = 0, k = 0; i < x; i = i + 10, k++)
    {
      stroke(0);
      strokeWeight(1);

      if (k % 5 == 0)
      {
        line(i, 0, i, 15);
      }
      else
      {
        line(i, 0, i, 10);
      }
    }

    for (let j = 0, k = 0; j < y; j = j + 10, k++)
    {
      stroke(0);
      strokeWeight(1);

      if (k % 5 == 0)
      {
        line(0, j, 15, j);
      }
      else
      {
        line(0, j, 10, j);
      }
    }

    stroke(255, 0, 0); // Red
    strokeWeight(10);
    line(0, 0, x, 0);

    stroke(0, 255, 0); // Blue
    strokeWeight(10);
    line(0, 0, 0, y);

    noStroke();
  }
}

function radians(deg)
{
  return deg * Math.PI / 180;
}
