class Robot
{
  constructor(robot_radius, x, y, theta, scan_range, scan_dist)
  {
    this.radius = robot_radius;

    this.x = x;
    this.y = y;
    this.theta = theta;

    this.scan_range = scan_range;
    this.scan_dist = scan_dist;
    this.scan_data = Array(Array(), Array());
 }

  odom_update(lin_vel, ang_vel, dt)
  {
    var delta_s = lin_vel * dt;
    var delta_theta = ang_vel * dt;

    this.x += delta_s * cos(this.theta + (delta_theta / 2.0));
    this.y += delta_s * sin(this.theta + (delta_theta / 2.0));
    this.theta += delta_theta;

    fill(0);
    text('x : ' + this.x, width - 50, height - 60);
    text('y : ' + this.y, width - 50, height - 40);
    text('theta : ' + this.theta, width - 50, height - 20);
  }

  scan_update(obstacle)
  {
    this.line_intersection = function(x1, y1, x2, y2, x3, y3, x4, y4)
    {
      let bx = x2 - x1;
      let by = y2 - y1;
      let dx = x4 - x3;
      let dy = y4 - y3;

      let b_dot_d_perp = bx * dy - by * dx;

      if (b_dot_d_perp == 0) return Array(-1.0, -1.0);

      let cx = x3 - x1;
      let cy = y3 - y1;

      let t = (cx * dy - cy * dx) / b_dot_d_perp;
      if (t < 0 || t > 1) return Array(-1.0, -1.0);

      let u = (cx * by - cy * bx) / b_dot_d_perp;
      if (u < 0 || u > 1) return Array(-1.0, -1.0);

      return Array(x1+t*bx, y1+t*by);
    }

    for (var angle = scan_range[0], count = 0; angle < scan_range[1]; angle = angle + radians(5.0), count++)
    {
      push();
      translate(this.x, this.y);
      rotate(this.theta);
      this.scan_data[count].push(
        this.line_intersection(
          0.0,
          0.0,
          cos(angle) * this.scan_dist,
          sin(angle) * this.scan_dist,

          ));
      pop();
    }
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

    // draw scan data
    stroke(255, 255, 0);
    strokeWeight(1);
    for (var i = 0; i < this.scan_data.length; i++)
    {
      line(this.x, this.y, this.x + this.scan_data, this.y + this.scan_data);
    }
  }
}

class Obstacle
{
  constructor(x1, y1, x2, y2, x3, y3, x4, y4)
  {
    this.x1 = x1;
    this.x2 = x2;
    this.x3 = x3;
    this.x4 = x4;

    this.y1 = y1;
    this.y2 = y2;
    this.y3 = y3;
    this.y4 = y4;

    this.color = (random(0, 255), random(0, 255), random(0, 255));
  }

  show()
  {
    noStroke();
    fill(this.color);
    quad(this.x1, this.y1, this.x2, this.y2, this.x3, this.y3, this.x4, this.y4);
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

function radians(deg)
{
  return deg * Math.PI / 180;
}

// robot config
let robot_radius = 10.0;
let x_in = 50.0;
let y_in = 50.0;
let theta_in = 0.0;
let scan_range = [radians(-90.0), radians(90.0)];
let scan_dist = 100.0;

var axis = new Axis();
var robot = new Robot(robot_radius, x_in, y_in, theta_in, scan_range, scan_dist);
var obstacle = [];

var t = 0;
var interval = 50;

var lin_vel = 0.0;
var ang_vel = 0.0;

function setup()
{
  createCanvas(400, 400);

  obstacle[0] = new Obstacle(100, 100, 100, 150, 150, 150, 150, 100);
}

function draw()
{
  if (millis() - t > interval)
  {
    axis.show(width, height);

    for (var i = 0; i < obstacle.length; i++)
    {
      obstacle[i].show();
    }

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
