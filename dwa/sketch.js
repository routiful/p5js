class Robot
{
  constructor(radius, x, y, theta)
  {
    this.radius = radius;
    this.x = x;
    this.y = y;
    this.theta = theta;

    this.lin_vel = 0.0;
    this.ang_vel = 0.0;
  }

  motion_update(lin_vel, ang_vel, duration, k = 1)
  {
    this.x = lin_vel * (k * duration * cos(ang_vel * (k * duration / 2.0)));
    this.y = lin_vel * (k * duration * sin(ang_vel * (k * duration / 2.0)))
  }

  move()
  {
    stroke(255, 0, 255);
    strokeWeight(1);
    fill(0);
    ellipse(this.x, this.y, this.radius, this.radius);

    stroke(255, 0, 255);
    strokeWeight(1);
    line(this.x, this.y, this.x + cos(this.theta) * (this.radius / 2), this.y + sin(this.theta) * (this.radius / 2));
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

function keyPressed() {
  if (key == 'w')
  {
    lin_vel = lin_vel + 0.1;
  }
  else if (key == 'x')
  {
    lin_vel = lin_vel - 0.1;
  }
  else if (key == 'a')
  {
    ang_vel = ang_vel + 0.1;
  }
  else if (key == 'd')
  {
    ang_vel = ang_vel - 0.1;
  }
  else if (key == 's')
  {
    lin_vel = 0.0;
    ang_vel = 0.0;
  }
}

var axis = new Axis();
var robot = new Robot(20, 50, 50, 0);

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

    robot.move();

    t = millis();
  }
}
