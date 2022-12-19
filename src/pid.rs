#[derive(Debug, Default)]
pub struct Pid {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
    pub tau: f64,
    pub lim_max: f64,
    pub lim_min: f64,
    pub lim_max_integrator: f64,
    pub lim_min_integrator: f64,
    pub sample_time: f64,
    pub integrator: f64,
    pub prev_err: f64,
    pub differentiator: f64,
    pub prev_measurement: f64,
    pub out: f64,
}

impl Pid {
    pub fn new(
        kp: f64,
        ki: f64,
        kd: f64,
        sample_time: f64,
        tau: f64,
        lim_min: f64,
        lim_max: f64,
        lim_min_integrator: f64,
        lim_max_integrator: f64,
    ) -> Self {
        Self {
            kp,
            ki,
            kd,
            tau,
            lim_max,
            lim_min,
            lim_max_integrator,
            lim_min_integrator,
            sample_time,
            integrator: 0.,
            prev_err: 0.,
            differentiator: 0.,
            prev_measurement: 0.,
            out: 0.,
        }
    }
    pub fn update(&mut self, set_point: f64, measurement: f64) {
        let error = set_point - measurement;
        let propotional = self.kp * error;

        self.integrator =
            self.integrator + 0.5_f64 * self.ki * self.sample_time * (error + self.prev_err);

        if self.integrator > self.lim_max_integrator {
            self.integrator = self.lim_max_integrator;
        } else if self.integrator < self.lim_min_integrator {
            self.integrator = self.lim_min_integrator;
        }
        self.differentiator = -(2.0_f64 * self.kd * (measurement - self.prev_measurement)
            + (2.0_f64 * self.tau - self.sample_time) * self.differentiator)
            / (2.0_f64 * self.tau + self.sample_time);

        self.out = propotional + self.integrator + self.differentiator;
        if self.out > self.lim_max {
            self.out = self.lim_max;
        } else if self.out < self.lim_min {
            self.out = self.lim_min;
        }
        self.prev_err = error;
        self.prev_measurement = measurement;
    }
}
