use genetic_algorithm::{Chromosome, Individual};
use neural_net::{Network, NeuronsInLayer};

const G: f32 = 9.81;
const ACC_MAX: f32 = 3.0;
const DRAG_COEFF: f32 = 1e-2; // must be positive
                              // 4 inputs are the 3 states and the height goal, 1 output is the acceleration
const TOPOLOGY: &[NeuronsInLayer] = &[NeuronsInLayer(4), NeuronsInLayer(8), NeuronsInLayer(1)];

#[derive(Default)]
struct State {
    pos_x: f32,
    pos_y: f32,
    vel_x: f32,
}

pub struct Cart {
    engine: Network,
    state: State,
    max_height_reached: f32, // fitness value
}

impl Cart {
    pub fn random(rng: &mut dyn common::RngCore) -> Self {
        Self {
            engine: Network::random(rng, TOPOLOGY),
            state: State::default(),
            max_height_reached: 0.0,
        }
    }

    pub fn compute_input_acceleration(&self, height_goal: f32) -> f32 {
        let nn_input = vec![
            self.state.pos_x,
            self.state.pos_y,
            self.state.vel_x,
            height_goal,
        ];

        let nn_output = self.engine.propagate(nn_input);
        // nn output should be a vec with a single element
        nn_output[0].min(ACC_MAX).max(-ACC_MAX)
    }

    pub fn propagate_dynamics(&mut self, input_acceleration: f32, dt: f32) {
        let (sin, cos) = (2.0 * self.state.pos_x).atan().sin_cos();
        let acc =
            input_acceleration - G * sin - self.state.vel_x.abs() * self.state.vel_x * DRAG_COEFF;

        self.state.pos_x += self.state.vel_x * dt;
        self.state.vel_x += acc * cos * dt; // cos: x direction only
        self.state.pos_y = self.state.pos_x * self.state.pos_x;

        // update "fitness" value
        self.max_height_reached = self.max_height_reached.max(self.state.pos_y);
    }
}

pub struct CartIndividual {
    fitness: f32,
    chromosome: Chromosome,
}

impl CartIndividual {
    pub fn from_cart(cart: &Cart) -> Self {
        Self {
            fitness: cart.max_height_reached,
            chromosome: cart.engine.weights().collect(),
        }
    }

    pub fn into_cart(self) -> Cart {
        Cart {
            engine: Network::from_weights(self.chromosome.into_iter(), TOPOLOGY),
            state: State::default(),
            max_height_reached: 0.0,
        }
    }
}

impl Individual for CartIndividual {
    fn fitness(&self) -> f32 {
        self.fitness
    }

    fn chromosome(&self) -> &Chromosome {
        &self.chromosome
    }

    fn create(chromosome: Chromosome) -> Self {
        Self {
            fitness: 0.0,
            chromosome,
        }
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
        let mut cart = Cart::random(&mut rng);

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
        let mut cart = Cart::random(&mut rng);

        for _ in 0..1000 {
            let input_acc = cart.compute_input_acceleration(5.0);
            cart.propagate_dynamics(input_acc, 1e-2);
        }

        assert_relative_eq!(cart.state.pos_x, 0.09211952);
        assert_relative_eq!(cart.state.pos_y, 0.008486006);
        assert_relative_eq!(cart.state.vel_x, -0.027811058);
    }

    #[test]
    fn convert_carts() {
        let mut rng = Cc8::from_seed(Default::default());
        let mut cart = Cart::random(&mut rng);

        cart.max_height_reached = 2.0;

        let cart_individual = CartIndividual::from_cart(&cart);
        assert_relative_eq!(cart_individual.fitness(), 2.0);
        let original_weights = cart.engine.weights().collect::<Vec<f32>>();
        let expected_weights = cart_individual
            .chromosome()
            .iter()
            .cloned()
            .collect::<Vec<f32>>();
        assert_relative_eq!(original_weights.as_slice(), expected_weights.as_slice());
    }
}
