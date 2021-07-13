use super::neuron::Neuron;

pub struct Layer {
    pub neurons: Vec<Neuron>,
}

impl Layer {
    pub fn random(
        rng: &mut dyn rand::RngCore,
        input_neurons: usize,
        output_neurons: usize,
    ) -> Self {
        let neurons = (0..output_neurons)
            .map(|_| Neuron::random(rng, input_neurons))
            .collect();
        Self { neurons }
    }

    pub fn propagate(&self, inputs: Vec<f32>) -> Vec<f32> {
        self.neurons
            .iter()
            .map(|neuron| neuron.propagate(&inputs))
            .collect()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    use approx::assert_relative_eq;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng as Cc8;

    #[test]
    fn random_layer() {
        let mut rng = Cc8::from_seed(Default::default());
        let layer = Layer::random(&mut rng, 4, 3);

        assert_eq!(layer.neurons.len(), 3);
        assert_eq!(layer.neurons[0].weights.len(), 4);
        assert_eq!(layer.neurons[1].weights.len(), 4);
        assert_eq!(layer.neurons[2].weights.len(), 4);
    }

    #[test]
    fn propagate_layer() {
        // 2 outputs and 3 inputs
        let layer = Layer {
            neurons: vec![
                Neuron {
                    bias: 0.5,
                    weights: vec![0.1, 0.2, 0.3],
                },
                Neuron {
                    bias: 0.1,
                    weights: vec![-0.5, 0.5, -0.5],
                },
            ],
        };

        let output = layer.propagate(vec![0.3, 0.2, -0.1]);
        assert_relative_eq!(output.as_slice(), [0.54, 0.1].as_ref());
    }
}
