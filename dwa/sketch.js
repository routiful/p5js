class Robot
{
  constructor(robot_radius, x, y, theta, scan_range, scan_offset, scan_dist)
  {
    this.radius = robot_radius;

    this.x = x;
    this.y = y;
    this.theta = theta;

    this.scan_range = scan_range;
    this.scan_dist = scan_dist;
    this.scan_offset = scan_offset;
    this.scan_data = [];
 }

  odom_update(lin_vel, ang_vel, dt)
  {
    var delta_s = lin_vel * dt;
    var delta_theta = ang_vel * dt;

    this.x += delta_s * cos(this.theta + (delta_theta / 2.0));
    this.y += delta_s * sin(this.theta + (delta_theta / 2.0));
    this.theta += delta_theta;

    fill(0);
    textSize(8);
    text('x : ' + this.x, width - 80, height - 40);
    text('y : ' + this.y, width - 80, height - 30);
    text('theta : ' + this.theta, width - 80, height - 20);

    text('lin_vel : ' + lin_vel, width - 80, height - 60);
    text('ang_vel : ' + ang_vel, width - 80, height - 50);
  }

  scan_update(obstacles)
  {
    // ref) https://www.openprocessing.org/sketch/135314/
    this.line_intersection = function(x1, y1, x2, y2, x3, y3, x4, y4)
    {
      var bx = x2 - x1;
      var by = y2 - y1;
      var dx = x4 - x3;
      var dy = y4 - y3;

      var b_dot_d_perp = bx * dy - by * dx;

      if (b_dot_d_perp == 0) return max(width, height);

      var cx = x3 - x1;
      var cy = y3 - y1;

      var t = (cx * dy - cy * dx) / b_dot_d_perp;
      if (t < 0 || t > 1) return max(width, height);

      var u = (cx * by - cy * bx) / b_dot_d_perp;
      if (u < 0 || u > 1) return max(width, height);

      var intersection_point_x = x1+t*bx;
      var intersection_point_y = y1+t*by;

      return sqrt(
        (this.x - intersection_point_x) * (this.x - intersection_point_x) +
        (this.y - intersection_point_y) * (this.y - intersection_point_y));

      // return {x: x1+t*bx, y: y1+t*by};
    }

    var cnt = 0;
    var obstacle_dist = [];
    var min_obstacle_dist = [];

    for (var i = this.scan_range[0]; i < this.scan_range[1]; i += this.scan_offset)
    {
      for (var j = 0; j < obstacles.length; j++)
      {
        obstacle_dist[0] =
          this.line_intersection(
            this.x,
            this.y,
            this.x + cos(this.theta + i) * this.scan_dist,
            this.y + sin(this.theta + i) * this.scan_dist,
            obstacles[j].x1,
            obstacles[j].y1,
            obstacles[j].x2,
            obstacles[j].y2
          );

        obstacle_dist[1] =
          this.line_intersection(
            this.x,
            this.y,
            this.x + cos(this.theta + i) * this.scan_dist,
            this.y + sin(this.theta + i) * this.scan_dist,
            obstacles[j].x2,
            obstacles[j].y2,
            obstacles[j].x3,
            obstacles[j].y3
          );

        obstacle_dist[2] =
          this.line_intersection(
            this.x,
            this.y,
            this.x + cos(this.theta + i) * this.scan_dist,
            this.y + sin(this.theta + i) * this.scan_dist,
            obstacles[j].x3,
            obstacles[j].y3,
            obstacles[j].x4,
            obstacles[j].y4
          );

        obstacle_dist[3] =
          this.line_intersection(
            this.x,
            this.y,
            this.x + cos(this.theta + i) * this.scan_dist,
            this.y + sin(this.theta + i) * this.scan_dist,
            obstacles[j].x4,
            obstacles[j].y4,
            obstacles[j].x1,
            obstacles[j].y1
          );

        min_obstacle_dist[j] = min(obstacle_dist);
      }

      min_obstacle_dist.push(
        this.line_intersection(
          this.x,
          this.y,
          this.x + cos(this.theta + i) * this.scan_dist,
          this.y + sin(this.theta + i) * this.scan_dist,
          0.0,
          0.0,
          width,
          0.0
        ));

      min_obstacle_dist.push(
        this.line_intersection(
          this.x,
          this.y,
          this.x + cos(this.theta + i) * this.scan_dist,
          this.y + sin(this.theta + i) * this.scan_dist,
          0.0,
          0.0,
          0.0,
          height
        ));

      min_obstacle_dist.push(
        this.line_intersection(
          this.x,
          this.y,
          this.x + cos(this.theta + i) * this.scan_dist,
          this.y + sin(this.theta + i) * this.scan_dist,
          width,
          0.0,
          width,
          height
        ));


      min_obstacle_dist.push(
        this.line_intersection(
          this.x,
          this.y,
          this.x + cos(this.theta + i) * this.scan_dist,
          this.y + sin(this.theta + i) * this.scan_dist,
          0.0,
          height,
          width,
          height
        ));

      console.log('min_obstacle_dist[' + min_obstacle_dist.length + ']: ' + min_obstacle_dist);
      this.scan_data[cnt] = min(min_obstacle_dist);
      // console.log('scan_data[' + cnt + ']: ' + this.scan_data[cnt]);
      cnt++;
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

        // console.log('scan_data : ' + this.scan_data);
    // draw scan data
    for (var i = this.scan_range[0], j = 0; i < this.scan_range[1]; i += radians(10.0), j++)
    {
      push();
      translate(this.x, this.y);
      rotate(this.theta);
      strokeWeight(1);

      if (this.scan_data[j] >= max(width, height))
      {
        stroke(255, 255, 0);
        line(0.0, 0.0, cos(i) * this.scan_dist, sin(i) * this.scan_dist);
      }
      else
      {
        stroke(255, 0, 0);
        line(0.0, 0.0, cos(i) * this.scan_data[j], sin(i) * this.scan_data[j]);
      }
      pop();
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

    this.color = 150;
  }

  show()
  {
    // noStroke();
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
let scan_offset = radians(10.0);
let scan_dist = 100.0;

var axis = new Axis();
var robot = new Robot(robot_radius, x_in, y_in, theta_in, scan_range, scan_offset, scan_dist);
var obstacles = [];

var t = 0;
var interval = 50;

var lin_vel = 0.0;
var ang_vel = 0.0;

function setup()
{
  createCanvas(400, 400);

  // obstacles[0] = new Obstacle(100, 100, 100, 150, 150, 150, 150, 100);
  obstacles[0] = new Obstacle(55, 160, 116, 141, 120, 210, 49, 204);
  obstacles[1] = new Obstacle(155, 274, 253, 279, 216, 360, 144, 360);
  obstacles[2] = new Obstacle(264, 40, 347, 45, 358, 136, 264, 40);
  obstacles[3] = new Obstacle(215, 114, 265, 114, 265, 164, 215, 164);
  obstacles[4] = new Obstacle(134, 32, 188, 32, 158, 88, 134, 32);
}

function draw()
{
  if (millis() - t > interval)
  {
    axis.show(width, height);

    for (var i = 0; i < obstacles.length; i++)
    {
      obstacles[i].show();
    }

    robot.odom_update(lin_vel, ang_vel, interval / 1000);
    robot.scan_update(obstacles);
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
