class Robot
{
  constructor(robot_radius, x, y, theta)
  {
    this.radius = robot_radius;

    this.x = x;
    this.y = y;
    this.theta = theta;
  }

  odom_update(lin_vel, ang_vel, dt)
  {
    var delta_s = lin_vel * dt;
    var delta_theta = ang_vel * dt;

    this.x += delta_s * cos(this.theta + (delta_theta / 2.0));
    this.y += delta_s * sin(this.theta + (delta_theta / 2.0));
    this.theta += delta_theta;

    text(this.x, width - 50, height - 60);
    text(this.y, width - 50, height - 40);
    text(this.theta, width - 50, height - 20);
  }

  motion_predict()
  {

  }

  draw()
  {
    // draw robot
    stroke(255, 0, 255);
    strokeWeight(1);
    fill(0);
    ellipse(this.x, this.y, this.radius * 2, this.radius * 2);

    // draw robot heading
    stroke(255, 0, 255);
    strokeWeight(1);
    line(this.x, this.y, this.x + cos(this.theta) * this.radius, this.y + sin(this.theta) * this.radius);
  }
}

class Obstacle
{
  constructor(shape, size)
  {

  }
}


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

var axis = new Axis();
var robot = new Robot(10.0, 50.0, 50.0, 0.0);

var t = 0;
var interval = 50;

var lin_vel = 0.0;
var ang_vel = 0.0;

function setup()
{
  createCanvas(400, 400);
}

function draw()
{
  if (millis() - t > interval)
  {
    axis.show(width, height);

    robot.odom_update(lin_vel, ang_vel, interval / 1000);
    robot.draw();

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
