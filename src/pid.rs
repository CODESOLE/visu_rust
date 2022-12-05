#[derive(Debug, Default)]
pub struct Pid {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub tau: f32,
    pub lim_max: f32,
    pub lim_min: f32,
    pub lim_max_integrator: f32,
    pub lim_min_integrator: f32,
    pub sample_time: f32,
    pub integrator: f32,
    pub prev_err: f32,
    pub differentiator: f32,
    pub prev_measurement: f32,
    pub out: f32,
}

impl Pid {
    pub fn new(
        kp: f32,
        ki: f32,
        kd: f32,
        sample_time: f32,
        tau: f32,
        lim_min: f32,
        lim_max: f32,
        lim_min_integrator: f32,
        lim_max_integrator: f32,
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
    pub fn update(&mut self, set_point: f32, measurement: f32) -> f32 {
        let error = set_point - measurement;
        let propotional = self.kp * error;

        self.integrator =
            self.integrator + 0.5_f32 * self.ki * self.sample_time * (error + self.prev_err);

        if self.integrator > self.lim_max_integrator {
            self.integrator = self.lim_max_integrator;
        } else if self.integrator < self.lim_min_integrator {
            self.integrator = self.lim_min_integrator;
        }
        self.differentiator = -(2.0_f32 * self.kd * (measurement - self.prev_measurement)
            + (2.0_f32 * self.tau - self.sample_time) * self.differentiator)
            / (2.0_f32 * self.tau + self.sample_time);

        self.out = propotional + self.integrator + self.differentiator;
        if self.out > self.lim_max {
            self.out = self.lim_max;
        } else if self.out < self.lim_min {
            self.out = self.lim_min;
        }
        self.prev_err = error;
        self.prev_measurement = measurement;

        self.out
    }
}
