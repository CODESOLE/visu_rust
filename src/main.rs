pub mod pid;
use pid::*;

const SAMPLE_TIME_S: f32 = 0.01;
static mut OUTPUT: f32 = 0.;
const ALPHA: f32 = 0.02;
fn test_system_update(input: f32) -> f32 {
    unsafe {
        OUTPUT = (SAMPLE_TIME_S * input + OUTPUT) / (1.0_f32 + ALPHA * SAMPLE_TIME_S);
        OUTPUT
    }
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
    for t in 0u16..400 {
        let i = f32::from(t) * 0.01; // 400 * 0.01 = 4 second
        let measure = test_system_update(pid.out);
        pid.update(SET_POINT, measure);
        println!("{:<20} {:<20} {:<20}", i, measure, pid.out);
    }
}
