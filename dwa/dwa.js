class DWA
{
  constructor(
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
    clearance_bias)
  {
    this.max_lin_vel = max_lin_vel;
    this.min_lin_vel = min_lin_vel;

    this.max_ang_vel = max_ang_vel;
    this.min_ang_vel = min_ang_vel;

    this.limit_lin_acc = limit_lin_acc;
    this.limit_ang_acc = limit_ang_acc;

    this.vx_samples = vx_samples;
    this.vth_samples = vth_samples;

    this.sim_time = sim_time;
    this.dt = dt;

    this.heading_bias = heading_bias;
    this.velocity_bias = velocity_bias;
    this.clearance_bias = clearance_bias;
  }

  predict_motion(state, input, dt)
  {
    // state[] : x_pos, y_pos, theta, lin_vel, ang_vel
    // input[] : lin_vel, ang_vel

    let get_state = state.slice(); // Deep copy

    let epsilon = 0.000001;
    if (input[0] - get_state[3] > epsilon)
    {
      get_state[3] = get_state[3] + this.limit_lin_acc * dt;
    }
    else if (input[0] - get_state[3] < -epsilon)
    {
      get_state[3] = get_state[3] - this.limit_lin_acc * dt;
    }
    else
    {
      get_state[3] = input[0];
    }

    if (abs(this.lin_vel) < epsilon)
    {
      this.lin_vel = 0.0;
    }

    if (input[1] - get_state[4] > epsilon)
    {
      get_state[4] = get_state[4] + this.limit_ang_acc * dt;
    }
    else if (input[1] - get_state[4] < -epsilon)
    {
      get_state[4] = get_state[4] - this.limit_ang_acc * dt;
    }
    else
    {
      get_state[4] = input[1];
    }

    if (abs(this.ang_vel) < epsilon)
    {
      this.ang_vel = 0.0;
    }

    let delta_s = get_state[3] * dt;
    let delta_theta = get_state[4] * dt;

    get_state[0] += delta_s * cos(get_state[2] + (delta_theta / 2.0));
    get_state[1] += delta_s * sin(get_state[2] + (delta_theta / 2.0));
    get_state[2] += delta_theta;

    get_state[2] = normalize_angle(get_state[2]);

    if (get_state[0] <= 0.0)
    {
      get_state[0] = 0.0;
    }

    if (get_state[1] <= 0.0)
    {
      get_state[1] = 0.0;
    }

    return get_state;
  }

  update_search_space(state)
  {
    // update possible velocity
    let Vs = [this.min_lin_vel, this.max_lin_vel, this.min_ang_vel, this.max_ang_vel];

    // update dynamic window velocity
    let Vd = [
      state[3] - this.limit_lin_acc * this.dt * 1.0,
      state[3] + this.limit_lin_acc * this.dt * 1.0,
      state[4] - this.limit_ang_acc * this.dt * 1.0,
      state[4] + this.limit_ang_acc * this.dt * 1.0];

    // update resulting velocity
    let Vr = [
      max(Vd[0], Vs[0]),
      min(Vd[1], Vs[1]),
      max(Vd[2], Vs[2]),
      min(Vd[3], Vs[3])];

    return Vr;
  }

  maximizing_objective_function(state, Vr, goal_pose, robot_radius, obstacles, scan_range, scan_offset, scan_dist)
  {
    let lin_vel = 0.0;
    let ang_vel = 0.0;

    let heading_cost = 0.0;
    let velocity_cost = 0.0;
    let clearance_cost = 0.0;

    let max_cost = 0.0;

    let end_points = [];
    let max_cost_points = 0;

    for (let dvx = Vr[0], i = 0; dvx <= Vr[1]; dvx += ((Vr[1] - Vr[0]) / this.vx_samples), i++)
    {
      for (let dvth = Vr[2], j = 0; dvth <= Vr[3]; dvth += ((Vr[3] - Vr[2]) / this.vth_samples), j++)
      {
        let predicted_trajectory = this.predict_trajectory(state, dvx, dvth);
        end_points.push(
          [predicted_trajectory[predicted_trajectory.length - 1][0],
          predicted_trajectory[predicted_trajectory.length - 1][1],
          predicted_trajectory[predicted_trajectory.length - 1][2]
          ]);

        heading_cost = this.heading_bias * this.heading(predicted_trajectory, goal_pose);
        velocity_cost = this.velocity_bias * this.velocity(predicted_trajectory);
        clearance_cost = this.clearance_bias * this.clearance(
          predicted_trajectory,
          robot_radius,
          obstacles,
          scan_range,
          scan_offset,
          scan_dist);

        let cost_sum = heading_cost + velocity_cost + clearance_cost;

        if (max_cost <= cost_sum)
        {
          max_cost = cost_sum;
          max_cost_points = (i * (this.vth_samples + 1)) + j;
          lin_vel = dvx;
          ang_vel = dvth;
        }
      }
    }

    for (let i = 0; i < end_points.length; i++)
    {
      noStroke();
      if (i == max_cost_points)
      {
        fill(0, 0, 0, 255);
        ellipse(
          end_points[i][0],
          end_points[i][1],
          5,
          5);
        stroke(0, 0, 0, 255);
        line(
          state[0],
          state[1],
          end_points[i][0],
          end_points[i][1]);
      }
      else
      {
        fill(0, 0, 255, 50);
        ellipse(
          end_points[i][0],
          end_points[i][1],
          5,
          5);
        stroke(0, 0, 255, 50);
        line(
          state[0],
          state[1],
          end_points[i][0],
          end_points[i][1]);
      }
    }

    return [lin_vel, ang_vel];
  }

  predict_trajectory(state, vx, vth)
  {
    let trajectory = [];
    trajectory[0] = state;

    for (let t = 0.0, i = 0; t <= this.sim_time; t += this.dt, i += 1)
    {
      trajectory.push(this.predict_motion(trajectory[i], [vx, vth], this.dt));
    }

    return trajectory;
  }

  heading(trajectory, goal_pose)
  {
    let dx = goal_pose[0] - trajectory[trajectory.length - 1][0];
    let dy = goal_pose[1] - trajectory[trajectory.length - 1][1];
    let diff_angle = normalize_angle(atan2(dy, dx));
    let error_angle = normalize_angle(diff_angle - trajectory[trajectory.length - 1][2]);

    let cost = 0.0;
    if (error_angle > 0.0)
    {
      cost = map(error_angle, 0.0, Math.PI, 1.0, 0.0);
    }
    else if (error_angle < 0.0)
    {
      cost = map(error_angle, -Math.PI, 0.0, 0.0, 1.0);
    }
    else
    {
      cost = 1.0;
    }
    return cost;
  }

  velocity(trajectory)
  {
    let error = abs(this.max_lin_vel - trajectory[trajectory.length - 1][3]);
    let cost = map(error, 0.0, this.max_lin_vel, 1.0, 0.0);

    return cost;
  }

  clearance(trajectory, robot_radius, obstacles, scan_range, scan_offset, scan_dist)
  {
    let x = trajectory[trajectory.length - 1][0];
    let y = trajectory[trajectory.length - 1][1];
    let theta = trajectory[trajectory.length - 1][2];

    this.check_inside = function()
    {
      // ray-casting algorithm based on
      // http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html

      // var x = point[0], y = point[1];
      let vs = obstacles;
      let inside = false;

      for (let i = 0, j = vs.length - 1; i < vs.length; j = i++)
      {
        let xi = vs[i][0], yi = vs[i][1];
        let xj = vs[j][0], yj = vs[j][1];

        let intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (intersect)
        {
          inside = !inside;
        }
      }
      return inside;
    }

    // ref) https://www.openprocessing.org/sketch/135314/
    this.line_intersection = function(x1, y1, x2, y2, x3, y3, x4, y4)
    {
      let bx = x2 - x1;
      let by = y2 - y1;
      let dx = x4 - x3;
      let dy = y4 - y3;

      let b_dot_d_perp = bx * dy - by * dx;

      if (b_dot_d_perp == 0) return scan_dist;

      let cx = x3 - x1;
      let cy = y3 - y1;

      let t = (cx * dy - cy * dx) / b_dot_d_perp;
      if (t < 0 || t > 1) return scan_dist;

      let u = (cx * by - cy * bx) / b_dot_d_perp;
      if (u < 0 || u > 1) return scan_dist;

      let intersection_point_x = x1+t*bx;
      let intersection_point_y = y1+t*by;

      return sqrt(
        pow((x - intersection_point_x), 2) +
        pow((y - intersection_point_y), 2));
    }

    this.predict_scan_data = function()
    {
      let cnt = 0;
      let scan_data = [];

      for (let i = scan_range[0]; i < scan_range[1]; i += scan_offset)
      {
        let min_obstacle_dist = [];
        for (let j = 0; j < obstacles.length; j++)
        {
          let obstacle_dist = [];
          obstacle_dist.push(
            this.line_intersection(
              x,
              y,
              x + cos(theta + i) * scan_dist,
              y + sin(theta + i) * scan_dist,
              obstacles[j].x1,
              obstacles[j].y1,
              obstacles[j].x2,
              obstacles[j].y2
            ));

          obstacle_dist.push(
            this.line_intersection(
              x,
              y,
              x + cos(theta + i) * scan_dist,
              y + sin(theta + i) * scan_dist,
              obstacles[j].x2,
              obstacles[j].y2,
              obstacles[j].x3,
              obstacles[j].y3
            ));

          obstacle_dist.push(
            this.line_intersection(
              x,
              y,
              x + cos(theta + i) * scan_dist,
              y + sin(theta + i) * scan_dist,
              obstacles[j].x3,
              obstacles[j].y3,
              obstacles[j].x4,
              obstacles[j].y4
            ));

          obstacle_dist.push(
            this.line_intersection(
              x,
              y,
              x + cos(theta + i) * scan_dist,
              y + sin(theta + i) * scan_dist,
              obstacles[j].x4,
              obstacles[j].y4,
              obstacles[j].x1,
              obstacles[j].y1
            ));

          min_obstacle_dist[j] = min(obstacle_dist);
        }

        min_obstacle_dist.push(
          this.line_intersection(
            x,
            y,
            x + cos(theta + i) * scan_dist,
            y + sin(theta + i) * scan_dist,
            0.0,
            0.0,
            width,
            0.0
          ));

        min_obstacle_dist.push(
          this.line_intersection(
            x,
            y,
            x + cos(theta + i) * scan_dist,
            y + sin(theta + i) * scan_dist,
            0.0,
            0.0,
            0.0,
            height
          ));

        min_obstacle_dist.push(
          this.line_intersection(
            x,
            y,
            x + cos(theta + i) * scan_dist,
            y + sin(theta + i) * scan_dist,
            width,
            0.0,
            width,
            height
          ));


        min_obstacle_dist.push(
          this.line_intersection(
            x,
            y,
            x + cos(theta + i) * scan_dist,
            y + sin(theta + i) * scan_dist,
            0.0,
            height,
            width,
            height
          ));

        scan_data[cnt] = min(min_obstacle_dist);
        cnt++;
      }
      return scan_data;
    }

    if (this.check_inside() == false)
    {
      let predicted_scan_data = this.predict_scan_data();

      let min_scan_data = min(predicted_scan_data);
      let cost = map(min_scan_data, robot_radius * 1.0, scan_dist, 0.0, 1.0);

      return cost;
    }
    else
    {
      return 0.0;
    }
  }
}
