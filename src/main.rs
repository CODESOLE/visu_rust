pub mod pid;
use pid::*;

const SAMPLE_TIME_S: f32 = 0.1;
const Y_K1: f32 = 0.;
const Y_K2: f32 = 0.;
const U_K1: f32 = 0.;
const U_K2: f32 = 0.;

fn plant_update(
    input: f32,
    iteration_count: u16,
    u_terms: &mut Vec<f32>,
    y_terms: &mut Vec<f32>,
) -> f32 {
    if iteration_count == 0 {
        let y = 0.004545 * (input + U_K2) + 0.009091 * U_K1 + 1.818 * Y_K1 - 0.8182 * Y_K2;
        u_terms.push(input);
        y_terms.push(y);
        return y;
    } else if iteration_count == 1 {
        let y = 0.004545 * (input + U_K1)
            + 0.009091 * u_terms[(iteration_count - 1) as usize]
            + 1.818 * y_terms[(iteration_count - 1) as usize]
            - 0.8182 * Y_K1;
        u_terms.push(input);
        y_terms.push(y);
        return y;
    } else {
        let y = 0.004545 * (input + u_terms[(iteration_count - 2) as usize])
            + 0.009091 * u_terms[(iteration_count - 1) as usize]
            + 1.818 * y_terms[(iteration_count - 1) as usize]
            - 0.8182 * y_terms[(iteration_count - 2) as usize];
        u_terms.push(input);
        y_terms.push(y);
        return y;
    }
}

fn main() {
    const SET_POINT: f32 = 1.;
    const PID_KP: f32 = 1.33;
    const PID_KI: f32 = 0.027;
    const PID_KD: f32 = 0.;
    const PID_TAU: f32 = 0.;
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

    let mut y_terms = Vec::<f32>::new();
    let mut u_terms = Vec::<f32>::new();
    let mut time = Vec::<f32>::new();

    println!(
        "{:<20} {:<20} {:<20}",
        "TIME", "SYS.OUT ( y[k] )", "PID.OUT ( u[k] )"
    );
    for t in 0u16..90 {
        // 90 * 0.1 = 9 seconds
        let i = f32::from(t) * 0.1;
        time.push(i);
        let measure = plant_update(pid.out, t, &mut u_terms, &mut y_terms);
        pid.update(SET_POINT, measure);
        println!("{:<20} {:<20} {:<20}", i, measure, pid.out);
    }
}
