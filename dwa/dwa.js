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

  update_dynamic_window(state, scan_data)
  {
    // update possible velocity
    let Vs = [this.min_lin_vel, this.max_lin_vel, this.min_ang_vel, this.max_ang_vel];

    // update admissible velocity
    let Va = [];
    for (let i = 0; i < scan_data.length; i++)
    {
      Va.push([
        this.min_lin_vel,
        sqrt(2 * scan_data[i] * this.limit_lin_acc),
        this.min_ang_vel,
        sqrt(2 * scan_data[i] * this.limit_ang_acc)]);
    }

    // update dynamic window velocity
    let Vd = [
      state[3] - this.limit_lin_acc * this.dt,
      state[3] + this.limit_lin_acc * this.dt,
      state[4] - this.limit_ang_acc * this.dt,
      state[4] + this.limit_ang_acc * this.dt];

    // update resulting velocity
    let Vr = [];
    for (let i = 0; i < Va.length; i++)
    {
      Vr.push([
        max(Vd[0], max(Vs[0], Va[i][0])),
        min(Vd[1], min(Vs[1], Va[i][1])),
        max(Vd[2], max(Vs[2], Va[i][2])),
        min(Vd[3], min(Vs[3], Va[i][3]))]);
    }

    return Vr;
  }

  update_velocity_input(state, Vr, goal_pose, scan_data)
  {
    let lin_vel;
    let ang_vel;



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