pub mod pid;
use pid::*;

use std::cell::RefCell;

const SAMPLE_TIME_S: f32 = 0.1;
const Y_K1: f32 = 0.;
const Y_K2: f32 = 0.;
const U_K1: f32 = 0.;
const U_K2: f32 = 0.;
thread_local!(static PAST_Y: RefCell<[f32; 2]> = RefCell::new([Y_K1, Y_K2]));
thread_local!(static PAST_U: RefCell<[f32; 2]> = RefCell::new([U_K1, U_K2]));

fn plant_update(input: f32, iteration_count: u16) -> f32 {
    if iteration_count == 0 {
        // let y2 =
    } else if iteration_count == 1 {
    } else {
    }
    unreachable!();
}

fn main() {
    const SET_POINT: f32 = 1.;
    const PID_KP: f32 = 2.;
    const PID_KI: f32 = 0.5;
    const PID_KD: f32 = 0.25;
    const PID_TAU: f32 = 0.02;
    const PID_LIM_MIN: f32 = -10.;
    const PID_LIM_MAX: f32 = 10.;
    const PID_LIM_MIN_INT: f32 = -5.;
    const PID_LIM_MAX_INT: f32 = 5.;

    let mut pid = Pid::new(
        PID_KP,
        PID_KI,
        PID_KD,
        SAMPLE_TIME_S,
        PID_TAU,
        PID_LIM_MIN,
        PID_LIM_MAX,
        PID_LIM_MIN_INT,
        PID_LIM_MAX_INT,
    );

    println!("{:<20} {:<20} {:<20}", "TIME", "SYS.OUT", "PID.OUT");
    for t in 0u16..40 {
        let i = f32::from(t) * 0.01; // 40 * 0.1 = 4 second
        let measure = plant_update(pid.out, t);
        pid.update(SET_POINT, measure);
        println!("{:<20} {:<20} {:<20}", i, measure, pid.out);
    }
}
