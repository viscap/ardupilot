#include "Copter.h"

bool Copter::basic_takeoff_init(bool ignore_checks)
{
    pos_control.set_alt_target(0);
    return true;
}

void Copter::basic_takeoff_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || ap.throttle_zero) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        return;
    }

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

    // get pilot's desired throttle
//    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);
    pos_control.update_z_controller();

    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    //attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}