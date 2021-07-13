mod layer;
mod neuron;

use layer::Layer;

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

pub struct LayerTopology {
    pub neurons: usize,
}

#[cfg(test)]
mod test {
    use super::neuron::Neuron;
    use super::*;

    use approx::assert_relative_eq;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng as Cc8;

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
