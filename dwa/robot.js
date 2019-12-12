class Robot
{
  constructor(robot_radius, x, y, theta, scan_range, scan_offset, scan_dist)
  {
    this.radius = robot_radius;

    this.x = x;
    this.y = y;
    this.theta = theta;

    this.lin_vel = 0.0;
    this.ang_vel = 0.0;

    this.lin_acc = 0.0;
    this.ang_acc = 0.0;

    this.scan_range = scan_range;
    this.scan_dist = scan_dist;
    this.scan_offset = scan_offset;
    this.scan_data = [];
  }

  odom_update(lin_vel, ang_vel, lin_acc, ang_acc, dt)
  {
    this.lin_acc = lin_acc;
    this.ang_acc = ang_acc;

    let epsilon = 0.0001;
    if (lin_vel - this.lin_vel > epsilon)
    {
      this.lin_vel = this.lin_vel + this.lin_acc;
    }
    else if (lin_vel - this.lin_vel < -epsilon)
    {
      this.lin_vel = this.lin_vel - this.lin_acc;
    }
    else
    {
      this.lin_vel = lin_vel;
    }

    if (ang_vel - this.ang_vel > epsilon)
    {
      this.ang_vel = this.ang_vel + this.ang_acc;
    }
    else if (ang_vel - this.ang_vel < -epsilon)
    {
      this.ang_vel = this.ang_vel - this.ang_acc;
    }
    else
    {
      this.ang_vel = ang_vel;
    }

    let delta_s = this.lin_vel * dt;
    let delta_theta = this.ang_vel * dt;

    this.x += delta_s * cos(this.theta + (delta_theta / 2.0));
    this.y += delta_s * sin(this.theta + (delta_theta / 2.0));
    this.theta += delta_theta;
  }

  scan_update(obstacles)
  {
    // ref) https://www.openprocessing.org/sketch/135314/
    this.line_intersection = function(x1, y1, x2, y2, x3, y3, x4, y4)
    {
      let bx = x2 - x1;
      let by = y2 - y1;
      let dx = x4 - x3;
      let dy = y4 - y3;

      let b_dot_d_perp = bx * dy - by * dx;

      if (b_dot_d_perp == 0) return this.scan_dist;

      let cx = x3 - x1;
      let cy = y3 - y1;

      let t = (cx * dy - cy * dx) / b_dot_d_perp;
      if (t < 0 || t > 1) return this.scan_dist;

      let u = (cx * by - cy * bx) / b_dot_d_perp;
      if (u < 0 || u > 1) return this.scan_dist;

      let intersection_point_x = x1+t*bx;
      let intersection_point_y = y1+t*by;

      return sqrt(
        pow((this.x - intersection_point_x), 2) +
        pow((this.y - intersection_point_y), 2));
    }

    let cnt = 0;
    for (let i = this.scan_range[0]; i < this.scan_range[1]; i += this.scan_offset)
    {
      let min_obstacle_dist = [];
      for (let j = 0; j < obstacles.length; j++)
      {
        let obstacle_dist = [];
        obstacle_dist.push(
          this.line_intersection(
            this.x,
            this.y,
            this.x + cos(this.theta + i) * this.scan_dist,
            this.y + sin(this.theta + i) * this.scan_dist,
            obstacles[j].x1,
            obstacles[j].y1,
            obstacles[j].x2,
            obstacles[j].y2
          ));

        obstacle_dist.push(
          this.line_intersection(
            this.x,
            this.y,
            this.x + cos(this.theta + i) * this.scan_dist,
            this.y + sin(this.theta + i) * this.scan_dist,
            obstacles[j].x2,
            obstacles[j].y2,
            obstacles[j].x3,
            obstacles[j].y3
          ));

        obstacle_dist.push(
          this.line_intersection(
            this.x,
            this.y,
            this.x + cos(this.theta + i) * this.scan_dist,
            this.y + sin(this.theta + i) * this.scan_dist,
            obstacles[j].x3,
            obstacles[j].y3,
            obstacles[j].x4,
            obstacles[j].y4
          ));

        obstacle_dist.push(
          this.line_intersection(
            this.x,
            this.y,
            this.x + cos(this.theta + i) * this.scan_dist,
            this.y + sin(this.theta + i) * this.scan_dist,
            obstacles[j].x4,
            obstacles[j].y4,
            obstacles[j].x1,
            obstacles[j].y1
          ));

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

      // console.log('min_obstacle_dist[' + min_obstacle_dist.length + ']: ' + min_obstacle_dist);
      this.scan_data[cnt] = min(min_obstacle_dist);
      // console.log('scan_data[' + cnt + ']: ' + this.scan_data[cnt]);
      cnt++;
    }
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
    for (let i = this.scan_range[0], j = 0; i < this.scan_range[1]; i += this.scan_offset, j++)
    {
      push();
      translate(this.x, this.y);
      rotate(this.theta);
      strokeWeight(1);

      if (this.scan_data[j] >= this.scan_dist)
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