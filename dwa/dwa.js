class DWA
{
  constructor(
    max_lin_vel,
    min_lin_vel,
    max_ang_vel,
    min_ang_vel,
    limit_lin_acc,
    limit_ang_acc,
    v_reso,
    yawrate_reso,
    dt,
    predict_time,
    heading_cost_gain,
    velocity_cost_gain,
    clearance_cost_gain)
  {
    this.max_lin_vel = max_lin_vel;
    this.min_lin_vel = min_lin_vel;

    this.max_ang_vel = max_ang_vel;
    this.min_ang_vel = min_ang_vel;

    this.limit_lin_acc = limit_lin_acc;
    this.limit_ang_acc = limit_ang_acc;

    this.v_reso = v_reso;
    this.yawrate_reso = yawrate_reso;

    this.dt = dt;
    this.predict_time = predict_time;

    this.heading_cost_gain = heading_cost_gain;
    this.velocity_cost_gain = velocity_cost_gain;
    this.clearance_cost_gain = clearance_cost_gain;

    this.goal_trajectory = [];
  }

  motion_predict(state, input, dt)
  {
    // state[] : x_pos, y_pos, theta, lin_vel, ang_vel
    // input[] : lin_vel, ang_vel
    // dt : control period time

    state[2] += input[1] * dt;

    state[0] += input[0] * dt * cos(state[2]);
    state[1] += input[0] * dt * sin(state[2]);

    state[3] = input[0];
    state[4] = input[1];

    return state;
  }

  update_search_space(state, closest_obstacle_dist)
  {
    // update possible velocity
    let Vs = [this.min_lin_vel, this.max_lin_vel, this.min_ang_vel, this.max_ang_vel];

    // update admissible velocity
    let Va = [
      this.min_lin_vel,
      sqrt(2 * closest_obstacle_dist * this.limit_lin_acc),
      this.min_ang_vel,
      sqrt(2 * closest_obstacle_dist * this.limit_ang_acc)];

    // update dynamic window velocity
    let Vd = [
      state[3] - this.limit_lin_acc * this.dt,
      state[3] + this.limit_lin_acc * this.dt,
      state[4] - this.limit_ang_acc * this.dt,
      state[4] + this.limit_ang_acc * this.dt];

    // update resulting velocity
    let Vr = [
      max(Vd[0], max(Vs[0], Va[0])),
      min(Vd[1], min(Vs[1], Va[1])),
      max(Vd[2], max(Vs[2], Va[2])),
      min(Vd[3], min(Vs[3], Va[3]))];

    return Vr;
  }

  maximizing_objective_function(state, dt, Vr, goal_pose, scan_data)
  {
    let lin_vel;
    let ang_vel;

    let min_cost = Infinity;

    // for (let i = 0; i < )


    return [lin_vel, ang_vel];
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