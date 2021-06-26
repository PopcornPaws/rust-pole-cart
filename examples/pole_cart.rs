use pole_cart::KissScene;
use std::time::Instant;

fn main() {
    let mut scene = KissScene::default();

    let ms_per_frame: f32 = 17.0;
    let max_cart_position: f32 = 2.0;
    let max_pole_angle: f32 = 1.0;
    let mut cart_position: f32 = 0.0;
    let mut pole_angle: f32 = 1.0;
    let mut cart_velocity: f32 = 1.0;
    let mut pole_angular_velocity: f32 = 0.5;
    let mut last_render_time = Instant::now();
    loop {
        let current_time = Instant::now();
        let delta_render = current_time.duration_since(last_render_time).as_millis() as f32;
        if delta_render >= ms_per_frame {
            cart_position += cart_velocity * delta_render / 1000.0;
            pole_angle += pole_angular_velocity * delta_render / 1000.0;

            if cart_position.abs() > max_cart_position {
                cart_velocity *= -1.0;
            }

            if pole_angle.abs() > max_pole_angle {
                pole_angular_velocity *= -1.0;
            }

            last_render_time = current_time;
            if !scene.render(cart_position, pole_angle) {
                break;
            }
        }
    }
}
