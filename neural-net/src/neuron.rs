use rand::Rng;

pub struct Neuron {
    pub bias: f32,
    pub weights: Vec<f32>,
}

impl Neuron {
    pub fn random(rng: &mut dyn rand::RngCore, output_size: usize) -> Self {
        let bias = rng.gen_range(-1.0..=1.0);
        let weights = (0..output_size)
            .map(|_| rng.gen_range(-1.0..=1.0))
            .collect();

        Self { bias, weights }
    }

    pub fn propagate(&self, inputs: &[f32]) -> f32 {
        assert_eq!(inputs.len(), self.weights.len());
        let output = inputs
            .iter()
            .zip(&self.weights)
            .map(|(input, weight)| input * weight)
            .sum::<f32>();
        (output + self.bias).max(0.0)
    }

    pub fn from_weights(input_size: usize, weights: &mut dyn Iterator<Item = f32>) -> Self {
        let bias = weights.next().expect("got not enough weights");
        let weights = (0..input_size)
            .map(|_| weights.next().expect("got not enough weights"))
            .collect();

        Self { bias, weights }
    }
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
    fn successful_from_weights() {
        let mut weights = vec![0.5_f32, 2.0, 3.0, 4.0, -5.0].into_iter();

        let neuron = Neuron::from_weights(4, &mut weights);
        assert_relative_eq!(neuron.bias, 0.5);
        assert_relative_eq!(neuron.weights.as_slice(), [2.0, 3.0, 4.0, -5.0].as_ref());
    }

    #[test]
    #[should_panic(expected = "got not enough weights")]
    fn no_bias() {
        let mut weights = vec![].into_iter();
        Neuron::from_weights(3, &mut weights);
    }

    #[test]
    #[should_panic(expected = "got not enough weights")]
    fn no_weights() {
        let mut weights = vec![1.2_f32, 3.4, 5.6].into_iter();
        Neuron::from_weights(3, &mut weights);
    }
}
