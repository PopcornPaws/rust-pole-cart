use rand::Rng;

pub struct Network {
    layers: Vec<Layer>,
}

impl Network {
    pub fn random(rng: &mut dyn rand::RngCore, layers: &[LayerTopology]) -> Self {
        assert!(layers.len() > 1);

        let layers = layers
            .windows(2)
            .map(|layers| Layer::random(rng, layers[0].neurons, layers[1].neurons))
            .collect();
        Self { layers }
    }

    pub fn propagate(&self, inputs: Vec<f32>) -> Vec<f32> {
        self.layers
            .iter()
            .fold(inputs, |inputs, layer| layer.propagate(inputs))
    }
}

struct Layer {
    neurons: Vec<Neuron>,
}

impl Layer {
    fn random(rng: &mut dyn rand::RngCore, input_neurons: usize, output_neurons: usize) -> Self {
        let neurons = (0..output_neurons)
            .map(|_| Neuron::random(rng, input_neurons))
            .collect();
        Self { neurons }
    }

    fn propagate(&self, inputs: Vec<f32>) -> Vec<f32> {
        self.neurons
            .iter()
            .map(|neuron| neuron.propagate(&inputs))
            .collect()
    }
}

struct Neuron {
    bias: f32,
    weights: Vec<f32>,
}

impl Neuron {
    fn random(rng: &mut dyn rand::RngCore, output_size: usize) -> Self {
        let bias = rng.gen_range(-1.0..=1.0);
        let weights = (0..output_size)
            .map(|_| rng.gen_range(-1.0..=1.0))
            .collect();

        Self { bias, weights }
    }

    fn propagate(&self, inputs: &[f32]) -> f32 {
        assert_eq!(inputs.len(), self.weights.len());
        let output = inputs
            .iter()
            .zip(&self.weights)
            .map(|(input, weight)| input * weight)
            .sum::<f32>();
        (output + self.bias).max(0.0)
    }
}

pub struct LayerTopology {
    pub neurons: usize,
}

#[cfg(test)]
mod test {
    use super::*;

    use approx::assert_relative_eq;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng as Cc8;

    #[test]
    fn random_neuron() {
        let mut rng = Cc8::from_seed(Default::default());
        let neuron = Neuron::random(&mut rng, 4);

        assert_relative_eq!(neuron.bias, -0.6255188);
        assert_relative_eq!(
            neuron.weights.as_slice(),
            [0.67383957, 0.8181262, 0.26284897, 0.5238807].as_ref()
        );
    }

    #[test]
    fn propagate_neuron() {
        let neuron = Neuron {
            bias: 0.5,
            weights: vec![-0.3, 0.8],
        };

        assert_relative_eq!(neuron.propagate(&[-10.0, -10.0]), 0.0);
        assert_relative_eq!(
            neuron.propagate(&[0.5, 1.0]),
            0.5 + 0.5 * (-0.3) + 1.0 * 0.8
        );
    }

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

    #[test]
    fn random_network() {
        let mut rng = Cc8::from_seed(Default::default());
        let network = Network::random(
            &mut rng,
            &[
                LayerTopology { neurons: 5 },
                LayerTopology { neurons: 4 },
                LayerTopology { neurons: 3 },
                LayerTopology { neurons: 2 },
            ],
        );

        assert_eq!(network.layers[0].neurons.len(), 4);
        assert_eq!(network.layers[1].neurons.len(), 3);
        assert_eq!(network.layers[2].neurons.len(), 2);

        assert_eq!(network.layers[0].neurons[0].weights.len(), 5);
        assert_eq!(network.layers[0].neurons[1].weights.len(), 5);
        assert_eq!(network.layers[0].neurons[2].weights.len(), 5);
        assert_eq!(network.layers[0].neurons[3].weights.len(), 5);
    }

    #[test]
    fn propagate_network() {
        let network = Network {
            layers: vec![
                Layer {
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
                },
                Layer {
                    neurons: vec![Neuron {
                        bias: 0.7,
                        weights: vec![-0.9, -0.1],
                    }],
                },
            ],
        };

        let output = network.propagate(vec![0.3, 0.2, -0.1]);
        assert_relative_eq!(output.as_slice(), [0.204].as_ref());
    }
}
