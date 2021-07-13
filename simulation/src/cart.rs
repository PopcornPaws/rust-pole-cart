use common::RngCore;
use neural_net::{Network, NeuronsInLayer};

const G: f32 = 9.81;

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
    pub fn new(rng: &mut dyn RngCore, topology: &[NeuronsInLayer], parameters: Parameters) -> Self {
        Self {
            engine: Network::random(rng, topology),
            state: State::default(),
            parameters,
            max_height_reached: 0.0,
        }
    }

    pub fn propagate_dynamics(&mut self, input_acc: f32, dt: f32) {
        let (sin, cos) = (2.0 * self.state.pos_x).atan().sin_cos();
        // TODO input acc could be thresholded already
        let acc = input_acc
            .max(-self.parameters.acc_max)
            .min(self.parameters.acc_max)
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
mod test {}
