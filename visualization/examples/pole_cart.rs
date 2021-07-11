use pole_cart::{KissScene, State};
use std::time::Instant;

fn main() {
    let mut scene = KissScene::default();
    let mut state = State {
        cart_position: 0.0,
        cart_velocity: 0.0,
        pole_angle: 0.0,
        pole_angular_velocity: 0.0,
    };

    let ms_per_frame: f32 = 17.0;
    let mut last_render_time = Instant::now();
    let mut last_dynamics_time = Instant::now();

    loop {
        let current_time = Instant::now();
        let delta_render = current_time.duration_since(last_render_time).as_millis() as f32;

        let delta_dynamics = current_time.duration_since(last_dynamics_time).as_millis() as f32;
        if delta_dynamics >= 1e-2 {
            last_dynamics_time = current_time;
            state.propagate_dynamics(0.1, delta_dynamics / 1000.0);
        } else if delta_render >= ms_per_frame {
            last_render_time = current_time;
            if !scene.render(&state) {
                break;
            }
        }
    }
}
