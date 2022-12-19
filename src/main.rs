pub mod car_model;
pub mod pid;
use car_model::*;
use pid::*;

use plotlib::page::*;
use plotlib::repr::*;
use plotlib::style::*;
use plotlib::view::*;

fn main() {
    const SET_POINT: f64 = 15.;
    const SAMPLE_TIME_S: f64 = 0.1;
    const PID_KP: f64 = 1.33;
    const PID_KI: f64 = 0.027;
    const PID_KD: f64 = 0.;
    const PID_TAU: f64 = 0.;
    const PID_LIM_MIN: f64 = -10.;
    const PID_LIM_MAX: f64 = 10.;
    const PID_LIM_MIN_INT: f64 = -5.;
    const PID_LIM_MAX_INT: f64 = 5.;

    let pid = Pid::new(
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

    let mut car_front = CarModel::new(None, 30., 50., SET_POINT);
    let mut car_back = CarModel::new(Some(pid), 40., 0., SET_POINT);

    let mut time = Vec::<f64>::new();

    let mut graph_data_abs_pos_back = Vec::<(f64, f64)>::new();
    let mut graph_data_abs_pos_front = Vec::<(f64, f64)>::new();
    let mut graph_data_speed_back = Vec::<(f64, f64)>::new();
    let mut graph_data_speed_front = Vec::<(f64, f64)>::new();

    for t in 0..90 {
        // 90 * 0.1 = 9 seconds
        let i = f64::from(t as u16) * 0.1;
        time.push(i);
        let (speed_front, abs_pos_front) = car_front.update(100., i, 0.);
        let (speed_back, abs_pos_back) =
            car_back.update(car_front.pos_x - car_back.pos_x, i, speed_front);
        graph_data_abs_pos_back.push((time[t], abs_pos_back));
        graph_data_abs_pos_front.push((time[t], abs_pos_front));
        graph_data_speed_back.push((time[t], speed_back));
        graph_data_speed_front.push((time[t], speed_front));
    }

    let l1 = Plot::new(graph_data_abs_pos_back)
        .line_style(LineStyle::new().colour("#FF0000").linejoin(LineJoin::Round))
        .legend("Arkadaki Araç Mutlak Mesafesi".to_string());
    let l2 = Plot::new(graph_data_abs_pos_front)
        .line_style(LineStyle::new().colour("#00FF00").linejoin(LineJoin::Round))
        .legend("Öndeki Araç Mutlak Mesafesi".to_string());
    let l3 = Plot::new(graph_data_speed_back)
        .line_style(LineStyle::new().colour("#FF0000").linejoin(LineJoin::Round))
        .legend("Arkadaki Araç Hızı".to_string());
    let l4 = Plot::new(graph_data_speed_front)
        .line_style(LineStyle::new().colour("#00FF00").linejoin(LineJoin::Round))
        .legend("Öndeki Araç Hızı".to_string());

    let v = ContinuousView::new().add(l1).add(l2);
    let v2 = ContinuousView::new().add(l3).add(l4);

    let v = v.x_label("Time (seconds)").y_label("Absolute Position (m)");
    let v2 = v2.x_label("Time (seconds)").y_label("Speed (m/s)");

    Page::single(&v)
        .save("graph_abs_pos.svg")
        .expect("saving svg");
    Page::single(&v2)
        .save("graph_speed.svg")
        .expect("saving svg");
}
