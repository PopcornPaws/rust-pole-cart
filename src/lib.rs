use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::nalgebra::{Point3, Translation3, UnitQuaternion, Vector3};
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;

const POLE_HALF_LENGTH: f32 = 1.0;
const POLE_LENGTH: f32 = 2.0 * POLE_HALF_LENGTH;
const POLE_Z_SHIFT: f32 = POLE_HALF_LENGTH + 0.11;
const POLE_MASS: f32 = 0.1;
const CART_MASS: f32 = 1.0;
const ALL_MASS: f32 = CART_MASS + POLE_MASS;
const POLE_X_INERTIA: f32 = POLE_MASS * POLE_HALF_LENGTH / 3.0;
const G: f32 = 9.81;

pub struct KissScene {
    camera: ArcBall,
    window: Window,
    cart: SceneNode,
    pole: SceneNode,
    _ground: SceneNode,
}

impl Default for KissScene {
    fn default() -> Self {
        let eye = Point3::<f32>::new(10.0, 10.0, 5.0);
        let look_at = Point3::<f32>::new(0.1, 0.1, 0.1);

        let mut camera = ArcBall::new(eye, look_at);
        camera.set_up_axis(Vector3::<f32>::z());

        let mut window = Window::new("Pole-cart");
        window.set_background_color(0.9, 0.9, 0.9);
        window.set_light(Light::StickToCamera);

        let mut ground = window.add_cube(100.0, 100.0, 0.1);
        ground.set_local_translation(Translation3::<f32>::new(0.0, 0.0, -1.0));

        let mut cart = window.add_cube(0.8, 1.2, 0.2);
        cart.set_color(0.5, 0.1, 0.7);

        let mut pole = window.add_cube(0.05, 0.05, POLE_LENGTH);
        pole.set_color(0.1, 0.5, 0.4);
        pole.set_local_translation(Translation3::<f32>::new(0.0, 0.0, POLE_Z_SHIFT));

        Self {
            camera,
            window,
            cart,
            pole,
            _ground: ground,
        }
    }
}

impl KissScene {
    pub fn render(&mut self, cart_position: f32, pole_angle: f32) -> bool {
        self.cart
            .set_local_translation(Translation3::new(0.0, cart_position, 0.0));

        let (sa, ca) = pole_angle.sin_cos();
        let pole_y_pos = cart_position - POLE_Z_SHIFT * sa;
        let pole_z_pos = POLE_Z_SHIFT * ca;

        self.pole
            .set_local_translation(Translation3::new(0.0, pole_y_pos, pole_z_pos));
        self.pole
            .set_local_rotation(UnitQuaternion::<f32>::from_axis_angle(
                &Vector3::x_axis(),
                pole_angle,
            ));

        let cart_pos = self.cart.data().local_translation().vector;
        let cart_rot = self.cart.data().local_rotation();

        let origin = &Point3::from(cart_pos);
        let end_i = &Point3::from(cart_pos + cart_rot * Vector3::<f32>::x());
        let end_j = &Point3::from(cart_pos + cart_rot * Vector3::<f32>::y());
        let end_k = &Point3::from(cart_pos + cart_rot * Vector3::<f32>::z());

        self.window
            .draw_line(origin, end_i, &Point3::new(1.0, 0.0, 0.0));
        self.window
            .draw_line(origin, end_j, &Point3::new(0.0, 1.0, 0.0));
        self.window
            .draw_line(origin, end_k, &Point3::new(0.0, 0.0, 1.0));

        self.window.render_with_camera(&mut self.camera)
    }
}

#[derive(Default)]
pub struct State {
    cart_position: f32,
    cart_velocity: f32,
    pole_angle: f32,
    pole_angular_velocity: f32,
}

// returns cart position and pole angle
impl State {
    pub fn dynamics(&mut self, input_force: f32, dt: f32) {
        let cart_forces = 
        self.cart_position += self.cart_velocity * dt;
        self.cart_velocity += force * dt / ALL_MASS;
    }
}
