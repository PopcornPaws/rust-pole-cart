mod layer;
mod neuron;

use layer::Layer;

pub struct Network {
    layers: Vec<Layer>,
}

impl Network {
    pub fn random(rng: &mut dyn common::RngCore, topology: &[NeuronsInLayer]) -> Self {
        assert!(topology.len() > 1);

        let layers = topology
            .windows(2)
            .map(|layers| Layer::random(rng, layers[0].0, layers[1].0))
            .collect();
        Self { layers }
    }

    pub fn propagate(&self, inputs: Vec<f32>) -> Vec<f32> {
        self.layers
            .iter()
            .fold(inputs, |inputs, layer| layer.propagate(inputs))
    }

    pub fn weights(&self) -> impl Iterator<Item = f32> + '_ {
        self.layers
            .iter()
            .flat_map(|layer| layer.neurons.iter())
            .flat_map(|neuron| std::iter::once(&neuron.bias).chain(&neuron.weights))
            .cloned()
    }

    pub fn from_weights(
        topology: &[NeuronsInLayer],
        weights: impl IntoIterator<Item = f32>,
    ) -> Self {
        assert!(topology.len() > 1);

        let mut weights = weights.into_iter();

        let layers = topology
            .windows(2)
            .map(|layers| Layer::from_weights(layers[0].0, layers[1].0, &mut weights))
            .collect();

        if weights.next().is_some() {
            panic!("got too many weights");
        }

        Self { layers }
    }
}

pub struct NeuronsInLayer(pub usize);

#[cfg(test)]
mod test {
    use super::neuron::Neuron;
    use super::*;

    use approx::assert_relative_eq;
    use common::*;

    #[test]
    fn random_network() {
        let mut rng = Cc8::from_seed(Default::default());
        let network = Network::random(
            &mut rng,
            &[
                NeuronsInLayer(5),
                NeuronsInLayer(4),
                NeuronsInLayer(3),
                NeuronsInLayer(2),
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

        // check propagated output
        let output = network.propagate(vec![0.3, 0.2, -0.1]);
        assert_relative_eq!(output.as_slice(), [0.204].as_ref());

        // check weights method as well
        let weights = network.weights().collect::<Vec<f32>>();
        assert_relative_eq!(
            weights.as_slice(),
            [0.5_f32, 0.1, 0.2, 0.3, 0.1, -0.5, 0.5, -0.5, 0.7, -0.9, -0.1].as_ref()
        );
    }

    #[test]
    #[should_panic(expected = "assertion failed")]
    fn not_enough_layers() {
        Network::from_weights(&[NeuronsInLayer(4)], &mut vec![2.3_f32].into_iter());
    }

    #[test]
    #[should_panic(expected = "got too many weights")]
    fn too_many_weights() {
        Network::from_weights(
            &[NeuronsInLayer(4), NeuronsInLayer(3), NeuronsInLayer(2)],
            &mut vec![2.3_f32; 35].into_iter(),
        );
    }

    #[test]
    fn successful_from_weights() {
        let network = Network::from_weights(
            &[NeuronsInLayer(4), NeuronsInLayer(3), NeuronsInLayer(2)],
            &mut vec![2.3_f32; 23].into_iter(),
        );

        assert_eq!(network.layers.len(), 2);
        assert_eq!(network.layers[0].neurons.len(), 3);
        assert_eq!(network.layers[1].neurons.len(), 2);
    }
}
