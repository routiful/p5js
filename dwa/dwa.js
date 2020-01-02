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

    // this.all_computed_trajectory = [];
  }

  predict_motion(state, input, dt)
  {
    // state[] : x_pos, y_pos, theta, lin_vel, ang_vel
    // input[] : lin_vel, ang_vel

    let get_state = state.slice(); // Deep copy

    let epsilon = 0.000000001;
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

    // update admissible velocity
    // let Va = [];
    // let check_scan_data = false;
    // let check_obstacles = false;
    // let radius_circular_trajectory = 0.0;
    // let theta_circular_trajectory = 0.0;
    // let circular_trajectory = 0.0;

    // for (let i = 0; i < scan_data.length; i++)
    // {
    //   if (scan_data[i] < scan_dist)
    //   {
    //     check_scan_data = true;
    //   }
    // }

    // if (check_scan_data == false)
    // {
    //   Va = [
    //     this.min_lin_vel,
    //     this.max_lin_vel,
    //     this.min_ang_vel,
    //     this.max_ang_vel];
    // }
    // else
    // {
      // if (state[4] == 0.0)
      // {
      //   radius_circular_trajectory = scan_dist;
      //   theta_circular_trajectory = 1.0;
      // }
      // else
      // {
      //   radius_circular_trajectory = state[3] / state[4];
      //   theta_circular_trajectory = state[4] * this.dt;
      // }

      // circular_trajectory = radius_circular_trajectory * theta_circular_trajectory;

      // for (let i = 0; i < scan_data.length; i++)
      // {
      //   if (scan_data[i] <= circular_trajectory)
      //   {
      //     check_obstacles = true;
      //   }
      // }

      // if (check_obstacles == true)
      // {
      //   Va = [
      //     this.min_lin_vel,
      //     sqrt(2 * circular_trajectory * acc[0]),
      //     -1.0 * sqrt(2 * circular_trajectory * acc[1]),
      //     sqrt(2 * circular_trajectory * acc[1])];
      // }
      // else
      // {
      //   Va = [
      //     this.min_lin_vel,
      //     this.max_lin_vel,
      //     this.min_ang_vel,
      //     this.max_ang_vel];
      // }
    // }

    // update dynamic window velocity
    let Vd = [
      state[3] - this.limit_lin_acc * this.dt,
      state[3] + this.limit_lin_acc * this.dt,
      state[4] - this.limit_ang_acc * this.dt,
      state[4] + this.limit_ang_acc * this.dt];

    // update resulting velocity
    let Vr = [
      max(Vd[0], Vs[0]),
      min(Vd[1], Vs[1]),
      max(Vd[2], Vs[2]),
      min(Vd[3], Vs[3])];
    // let Vr = [
    //   max(Vd[0], max(Vs[0], Va[0])),
    //   min(Vd[1], min(Vs[1], Va[1])),
    //   max(Vd[2], max(Vs[2], Va[2])),
    //   min(Vd[3], min(Vs[3], Va[3]))];

    return Vr;
  }

  maximizing_objective_function(state, Vr, goal_pose, robot_radius, scan_data, scan_dist)
  {
    let lin_vel;
    let ang_vel;

    let heading_cost = 0.0;
    let velocity_cost = 0.0;
    let clearance_cost = 0.0;

    let max_cost = 0.0;

    let end_points = [];

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
        velocity_cost = 0.0; //this.velocity_bias * this.velocity(predicted_trajectory);
        clearance_cost = 0.0; //this.clearance_bias * this.clearance(predicted_trajectory, robot_radius, scan_data, scan_dist);

        let cost_sum = heading_cost + velocity_cost + clearance_cost;

        if (max_cost <= cost_sum)
        {
          max_cost = cost_sum;
          lin_vel = dvx;
          ang_vel = dvth;
        }
      }
    }

    for (let i = 0; i < end_points.length; i++)
    {
      noStroke();
      fill(0, 0, 255);
      ellipse(
        end_points[i][0],
        end_points[i][1],
        5,
        5);
      stroke(0, 0, 255);
      line(
        end_points[i][0],
        end_points[i][1],
        end_points[i][0] + cos(end_points[i][2]) * 20,
        end_points[i][1] + sin(end_points[i][2]) * 20);
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

  clearance(trajectory, robot_radius, scan_data, scan_dist)
  {
    // let min_scan_data = min(scan_data);
    // let cost = map(min_scan_data, robot_radius, scan_dist, 0.0, 1.0);

    let cost = 0.0;
    let x = 0.0;
    let y = 0.0;
    let dx = 0.0;
    let dy = 0.0;
    let diff = 0.0;

    for (let i = 0; i < scan_data.length; i++)
    {
      x = scan_data[i] *
        cos((scan_range[0] + scan_offset * i) + trajectory[trajectory.length - 1][2]) +
        trajectory[trajectory.length - 1][0];

      y = scan_data[i] *
      sin((scan_range[0] + scan_offset * i) + trajectory[trajectory.length - 1][2]) +
      trajectory[trajectory.length - 1][1];

      if (x < 0.0)
      {
        x = 0.0;
      }

      if (y < 0.0)
      {
        y = 0.0;
      }

      dx = trajectory[trajectory.length - 1][0] - x;
      dy = trajectory[trajectory.length - 1][1] - y;

      diff = sqrt(pow(dx, 2) + pow(dy, 2));
      if (diff<= robot_radius)
      {
        return cost = 0.0;
      }
    }

    return cost;
  }
}
