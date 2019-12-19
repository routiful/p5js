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

    let delta_s = input[0] * dt;
    let delta_theta = input[1] * dt;

    get_state[0] += delta_s * cos(get_state[2] + (delta_theta / 2.0));
    get_state[1] += delta_s * sin(get_state[2] + (delta_theta / 2.0));
    get_state[2] += delta_theta;
    get_state[2] = normalize_angle(get_state[2]);

    get_state[3] = input[0];
    get_state[4] = input[1];

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

  maximizing_objective_function(state, Vr, goal_pose, scan_data, scan_offset)
  {
    let lin_vel;
    let ang_vel;

    let cost_sum = [];

    for (let dvx = Vr[0], i = 0; dvx <= Vr[1]; dvx += ((Vr[1] - Vr[0]) / this.vx_samples), i++)
    {
      for (let dvth = Vr[2], j = 0; dvth <= Vr[3]; dvth += ((Vr[3] - Vr[2]) / this.vth_samples), j++)
      {
        let predicted_trajectory = this.predict_trajectory(state, dvx, dvth);
        // this.all_computed_trajectory.push(predicted_trajectory);

        // for (let i = 0; i < predicted_trajectory.length; i++)
        // {
        //   stroke(0);
        //   strokeWeight(3);
        //   point(predicted_trajectory[0], predicted_trajectory[1]);
        // }

        let heading_cost = this.heading_bias * this.heading(predicted_trajectory, goal_pose);
        let velocity_cost = this.velocity_bias * this.velocity(predicted_trajectory);
        let clearance_cost = this.clearance_bias * this.clearance(predicted_trajectory, scan_data, scan_offset);

        for (let k = 0; k < predicted_trajectory.length; k++)
        {
          cost_sum[i][j] = heading_cost[k] + velocity_cost[k] + clearance_cost[k];
        }

        // max_cost = max(cost_sum);

        // if (max_cost <= cost_sum)
        // {
        //   max_cost = cost_sum;
        //   lin_vel = dvx;
        //   ang_vel = dvth;
        // }
      }
    }

    let max_cost = cost_sum.indexOf(max(cost_sum));
    // return [lin_vel, ang_vel];
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
    let cost = [];

    for (let i = 0; i < trajectory.length; i++)
    {
      let error = abs(normalize_angle(goal_pose[2] - trajectory[i][2]));
      cost[i] = map(error, 0.0, Math.PI, 1.0, 0.0);
    }

    return cost;
  }

  velocity(trajectory)
  {
    let cost = [];

    for (let i = 0; i < trajectory.length; i++)
    {
      let error = abs(this.max_lin_vel - trajectory[i][3]);
      cost[i] = map(error, 0.0, this.max_lin_vel, 1.0, 0.0);
    }

    return cost;
  }

  clearance(trajectory, scan_data, scan_offset)
  {
    let cost = [];
    let point_cloud = [];
    let i = 0;
    let j = 0;

    for (i = 0; i < scan_data.length; i++)
    {
      point_cloud[i][0] = scan_data[i] * cos(Math.PI - scan_offset);
      point_cloud[i][1] = scan_data[i] * sin(Math.PI - scan_offset);
    }

    for (i = 0; i < trajectory.length; i++)
    {
      cost[i] = 0.0;
    }

    for (i = 0; i < trajectory.length; i++)
    {
      for (j = 0; j < scan_data.length; j++)
      {
        if (trajectory[i][0] == point_cloud[j][0] && trajectory[i][1] == point_cloud[j][1])
        {
          cost[i] = 1.0;
        }
      }
    }

    return cost;
  }
}
