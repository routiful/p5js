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

    let delta_s = input[0] * dt;
    let delta_theta = input[1] * dt;

    state[0] += delta_s * cos(state[2] + (delta_theta / 2.0));
    state[1] += delta_s * sin(state[2] + (delta_theta / 2.0));
    state[2] += delta_theta;
    state[2] = normalize_angle(state[2]);

    state[3] = input[0];
    state[4] = input[1];

    return new Array(state[0], state[1], state[2], state[3], state[4]);
  }

  update_search_space(state, closest_obstacle_dist, acc)
  {
    // update possible velocity
    let Vs = [this.min_lin_vel, this.max_lin_vel, this.min_ang_vel, this.max_ang_vel];

    // update admissible velocity
    let Va = [
      this.min_lin_vel,
      sqrt(2 * closest_obstacle_dist * acc[0]),
      this.min_ang_vel,
      sqrt(2 * closest_obstacle_dist * acc[1])];

    // update dynamic window velocity
    let Vd = [
      state[3] - acc[0] * this.dt,
      state[3] + acc[0] * this.dt,
      state[4] - acc[1] * this.dt,
      state[4] + acc[1] * this.dt];

    // update resulting velocity
    let Vr = [
      max(Vd[0], max(Vs[0], Va[0])),
      min(Vd[1], min(Vs[1], Va[1])),
      max(Vd[2], max(Vs[2], Va[2])),
      min(Vd[3], min(Vs[3], Va[3]))];

    return Vr;
  }

  maximizing_objective_function(state, Vr, goal_pose, scan_data)
  {
    let lin_vel;
    let ang_vel;

    let max_cost = 0.0;

    for (let dvx = Vr[0]; dvx <= Vr[1]; dvx += ((Vr[1] - Vr[0]) / this.vx_samples))
    {
      for (let dvth = Vr[2]; dvth <= Vr[3]; dvth += ((Vr[3] - Vr[2]) / this.vth_samples))
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
        // let velocity_cost = this.velocity_bias * this.velocity();
        // let clearance_cost = this.clearance_bias * this.clearance();

        // let cost_sum = heading_cost + velocity_cost + clearance_cost;

        // if (max_cost <= cost_sum)
        // {
        //   max_cost = cost_sum;
        //   lin_vel = dvx;
        //   ang_vel = dvth;
        // }
      }
    }
    // return [lin_vel, ang_vel];
  }

  predict_trajectory(state, vx, vth)
  {
    let trajectory = [];
    trajectory[0] = state;

    for (let t = 0.0, i = 0; t <= this.sim_time; t += this.dt, i += 1)
    {
      trajectory[i+1] = this.predict_motion(trajectory[i], [vx, vth], this.dt);
    }

    return trajectory;
  }

  heading(trajectory, goal_pose)
  {
    let cost = 0.0;
    let error = [];

    for (let i = 0; i < trajectory.length; i++)
    {
      error[i] = normalize_angle(goal_pose[2] - trajectory[i][2]);
    }

    let min = error.reduce(function(previous, current)
      {
        return previous > current ? current : previous;
      });

    cost = map(min, -Math.PI, Math.PI, 0.0, 1.0);
    return cost;
  }

  clearance()
  {

  }

  velocity()
  {

  }
}
