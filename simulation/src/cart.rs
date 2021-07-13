use neural_net::{Network, NeuronsInLayer};

const G: f32 = 9.81;

#[derive(Clone, Copy, Default)]
pub struct Parameters {
    pub acc_max: f32,
    pub drag_coeff: f32,
}

#[derive(Default)]
struct State {
    pos_x: f32,
    pos_y: f32,
    vel_x: f32,
}

impl State {
    fn as_vec(&self) -> Vec<f32> {
        vec![self.pos_x, self.pos_y, self.vel_x]
    }
}

pub struct Cart {
    engine: Network,
    state: State,
    parameters: Parameters,
    max_height_reached: f32, // fitness value
}

impl Cart {
    pub fn random(
        rng: &mut dyn common::RngCore,
        topology: &[NeuronsInLayer],
        parameters: Parameters,
    ) -> Self {
        assert_eq!(topology[0].0, 3); // 3 inputs are the states
        assert_eq!(topology[topology.len() - 1].0, 1); // 1 output is the acceleration command
        assert!(parameters.drag_coeff >= 0.0); // drag coefficient must be positive

        Self {
            engine: Network::random(rng, topology),
            state: State::default(),
            parameters,
            max_height_reached: 0.0,
        }
    }

    pub fn compute_input_acceleration(&self) -> f32 {
        let nn_output = self.engine.propagate(self.state.as_vec());
        // nn output should be a vec with a single element
        nn_output[0]
            .min(-self.parameters.acc_max)
            .max(self.parameters.acc_max)
    }

    pub fn propagate_dynamics(&mut self, input_acceleration: f32, dt: f32) {
        let (sin, cos) = (2.0 * self.state.pos_x).atan().sin_cos();
        let acc = input_acceleration
            - G * sin
            - self.state.vel_x.abs() * self.state.vel_x * self.parameters.drag_coeff;

        self.state.pos_x += self.state.vel_x * dt;
        self.state.vel_x += acc * cos * dt; // cos: x direction only
        self.state.pos_y = self.state.pos_x * self.state.pos_x;

        // update "fitness" value
        self.max_height_reached = self.max_height_reached.max(self.state.pos_y);
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use common::*;
    use std::f32::consts::PI;

    #[test]
    fn equilibrium_at_45_deg_slope() {
        let mut rng = Cc8::from_seed(Default::default());
        let mut cart = Cart::random(
            &mut rng,
            &[NeuronsInLayer(3), NeuronsInLayer(1)],
            Parameters::default(),
        );

        cart.state.pos_x = 0.5;

        let input_acceleration = G * (PI / 4.0).sin();

        for _ in 0..1000 {
            cart.propagate_dynamics(input_acceleration, 1e-1);
            // assert that we are not moving
            assert_relative_eq!(cart.state.pos_x, 0.5);
            assert_relative_eq!(cart.state.pos_y, 0.25);
            assert_relative_eq!(cart.state.vel_x, 0.0);
        }
    }

    #[test]
    fn propagate_random_dynamics() {
        let mut rng = Cc8::from_seed(Default::default());
        let mut cart = Cart::random(
            &mut rng,
            &[NeuronsInLayer(3), NeuronsInLayer(6), NeuronsInLayer(1)],
            Parameters {
                acc_max: 4.0,
                drag_coeff: 0.0,
            },
        );

        for _ in 0..1000 {
            let input_acc = cart.compute_input_acceleration();
            cart.propagate_dynamics(input_acc, 1e-2);
        }

        assert_relative_eq!(cart.state.pos_x, -0.070806265);
        assert_relative_eq!(cart.state.pos_y, 0.0050135273);
        assert_relative_eq!(cart.state.vel_x, -0.88140285);
    }
}
