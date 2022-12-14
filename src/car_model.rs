use crate::pid::*;

const Y_K1: f32 = 0.;
const Y_K2: f32 = 0.;
const U_K1: f32 = 0.;
const U_K2: f32 = 0.;

pub struct CarModel {
    pub ini_pos_x: f32,
    pub pos_x: f32,
    pub safe_distance: f32,
    pub free_mode_ref_speed: f32,
    pub pid: Pid,
    pub y_terms: Vec<f32>,
    pub u_terms: Vec<f32>,
    iteration_count: u16,
}

impl CarModel {
    pub fn new(
        pid: Option<Pid>,
        free_mode_set_speed: f32,
        ini_pos_x: f32,
        safe_distance: f32,
        st: f32,
    ) -> Self {
        if let Some(pidd) = pid {
            Self {
                ini_pos_x,
                pos_x: ini_pos_x,
                safe_distance,
                free_mode_ref_speed: free_mode_set_speed,
                pid: pidd,
                y_terms: Vec::<f32>::new(),
                u_terms: Vec::<f32>::new(),
                iteration_count: 0,
            }
        } else {
            Self {
                ini_pos_x,
                pos_x: ini_pos_x,
                safe_distance,
                free_mode_ref_speed: free_mode_set_speed,
                pid: Pid {
                    kp: 0.,
                    ki: 0.,
                    kd: 0.,
                    tau: 0.,
                    lim_max: 0.,
                    lim_min: 0.,
                    lim_max_integrator: 0.,
                    lim_min_integrator: 0.,
                    integrator: 0.,
                    prev_err: 0.,
                    sample_time: st,
                    differentiator: 0.,
                    prev_measurement: 0.,
                    out: 0.,
                },
                y_terms: Vec::<f32>::new(),
                u_terms: Vec::<f32>::new(),
                iteration_count: 0,
            }
        }
    }
    pub fn update(&mut self, front_car_distance: f32, t: f32, front_car_speed: f32) -> (f32, f32) {
        if front_car_distance > self.safe_distance {
            if self.iteration_count == 0 {
                let err = self.free_mode_ref_speed - Y_K2;
                let y = 0.004545 * (err + U_K2) + 0.009091 * U_K1 + 1.818 * Y_K1 - 0.8182 * Y_K2;
                self.u_terms.push(err);
                self.y_terms.push(y);

                self.iteration_count += 1;
                self.pos_x = y * t + self.ini_pos_x;
                (y, y * t + self.ini_pos_x)
            } else if self.iteration_count == 1 {
                let err = self.free_mode_ref_speed - Y_K1;
                let y = 0.004545 * (err + U_K1)
                    + 0.009091 * self.u_terms[(self.iteration_count - 1) as usize]
                    + 1.818 * self.y_terms[(self.iteration_count - 1) as usize]
                    - 0.8182 * Y_K1;
                self.u_terms.push(err);
                self.y_terms.push(y);

                self.iteration_count += 1;
                self.pos_x = y * t + self.ini_pos_x;
                (y, y * t + self.ini_pos_x)
            } else {
                let err =
                    self.free_mode_ref_speed - self.y_terms[(self.iteration_count - 2) as usize];

                let y = 0.004545 * (err + self.u_terms[(self.iteration_count - 2) as usize])
                    + 0.009091 * self.u_terms[(self.iteration_count - 1) as usize]
                    + 1.818 * self.y_terms[(self.iteration_count - 1) as usize]
                    - 0.8182 * self.y_terms[(self.iteration_count - 2) as usize];
                self.u_terms.push(err);
                self.y_terms.push(y);

                self.iteration_count += 1;
                self.pos_x = y * t + self.ini_pos_x;
                (y, y * t + self.ini_pos_x)
            }
        } else {
            let y = 0.004545 * (self.pid.out + self.u_terms[(self.iteration_count - 2) as usize])
                + 0.009091 * self.u_terms[(self.iteration_count - 1) as usize]
                + 1.818 * self.y_terms[(self.iteration_count - 1) as usize]
                - 0.8182 * self.y_terms[(self.iteration_count - 2) as usize];
            self.u_terms.push(self.pid.out);
            self.y_terms.push(y);

            self.pid.update(front_car_speed, y);
            self.pos_x = y * t + self.ini_pos_x;
            self.iteration_count += 1;
            (y, y * t + self.ini_pos_x)
        }
    }
}