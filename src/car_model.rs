use crate::pid::*;

const Y_K1: f64 = 0.;
const Y_K2: f64 = 0.;
const U_K1: f64 = 0.;
const U_K2: f64 = 0.;

pub struct CarModel {
    ini_pos_x: f64,
    pub pos_x: f64,
    safe_distance: f64,
    free_mode_ref_speed: f64,
    pid: Pid,
    y_terms: Vec<f64>,
    u_terms: Vec<f64>,
    iteration_count: usize,
}

impl CarModel {
    pub fn new(
        pid: Option<Pid>,
        free_mode_set_speed: f64,
        ini_pos_x: f64,
        safe_distance: f64,
    ) -> Self {
        if let Some(pidd) = pid {
            Self {
                ini_pos_x,
                pos_x: ini_pos_x,
                safe_distance,
                free_mode_ref_speed: free_mode_set_speed,
                pid: pidd,
                y_terms: Vec::<f64>::new(),
                u_terms: Vec::<f64>::new(),
                iteration_count: 0,
            }
        } else {
            Self {
                ini_pos_x,
                pos_x: ini_pos_x,
                safe_distance,
                free_mode_ref_speed: free_mode_set_speed,
                pid: Default::default(),
                y_terms: Vec::<f64>::new(),
                u_terms: Vec::<f64>::new(),
                iteration_count: 0,
            }
        }
    }
    pub fn update(&mut self, front_car_distance: f64, t: f64, front_car_speed: f64) -> (f64, f64) {
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
                    + 0.009091 * self.u_terms[self.iteration_count - 1]
                    + 1.818 * self.y_terms[self.iteration_count - 1]
                    - 0.8182 * Y_K1;
                self.u_terms.push(err);
                self.y_terms.push(y);

                self.iteration_count += 1;
                self.pos_x = y * t + self.ini_pos_x;
                (y, y * t + self.ini_pos_x)
            } else {
                let err = self.free_mode_ref_speed - self.y_terms[self.iteration_count - 2];

                let y = 0.004545 * (err + self.u_terms[self.iteration_count - 2])
                    + 0.009091 * self.u_terms[self.iteration_count - 1]
                    + 1.818 * self.y_terms[self.iteration_count - 1]
                    - 0.8182 * self.y_terms[self.iteration_count - 2];
                self.u_terms.push(err);
                self.y_terms.push(y);

                self.iteration_count += 1;
                self.pos_x = y * t + self.ini_pos_x;
                (y, y * t + self.ini_pos_x)
            }
        } else {
            self.pid
                .update(front_car_speed, *self.y_terms.last().unwrap());

            let y = 0.004545 * (self.pid.out + self.u_terms[self.iteration_count - 2])
                + 0.009091 * self.u_terms[self.iteration_count - 1]
                + 1.818 * self.y_terms[self.iteration_count - 1]
                - 0.8182 * self.y_terms[self.iteration_count - 2];

            self.u_terms.push(self.pid.out);
            self.y_terms.push(y);

            self.pos_x = y * t + self.ini_pos_x;
            self.iteration_count += 1;
            (y, y * t + self.ini_pos_x)
        }
    }
}
