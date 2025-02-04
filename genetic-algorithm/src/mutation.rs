use super::Chromosome;
use rand::{Rng, RngCore};

pub trait MutationMethod {
    fn mutate(&self, rng: &mut dyn RngCore, child: &mut Chromosome);
}

pub struct GaussianMutation {
    // Probability of changing a gene
    // 0.0 = no genes will be touched
    // 1.0 = all genes will be touched
    chance: f32,
    // Magnitude of that change:
    // 0.0 = touched genes will not be modified
    // 3.0 = touched genes will += or -= by at most 3.0
    coeff: f32,
}

impl GaussianMutation {
    pub fn new(chance: f32, coeff: f32) -> Self {
        Self { chance, coeff }
    }
}

impl MutationMethod for GaussianMutation {
    fn mutate(&self, rng: &mut dyn RngCore, child: &mut Chromosome) {
        for gene in child.iter_mut() {
            let sign = if rng.gen_bool(0.5) { -1.0 } else { 1.0 };
            if rng.gen_bool(self.chance as _) {
                *gene += sign * self.coeff * rng.gen::<f32>();
            }
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng as Cc8;

    fn actual(chance: f32, coeff: f32) -> Vec<f32> {
        let mut child = vec![1.0, 2.0, 3.0, 4.0, 5.0].into_iter().collect();

        let mut rng = Cc8::from_seed(Default::default());
        GaussianMutation::new(chance, coeff).mutate(&mut rng, &mut child);

        child.into_iter().collect()
    }

    mod given_zero_chance {
        fn actual(coeff: f32) -> Vec<f32> {
            super::actual(0.0, coeff)
        }

        mod and_zero_coefficient {
            #[test]
            fn does_not_change_the_original_chromosome() {
                let actual = super::actual(0.0);
                let expected = vec![1.0, 2.0, 3.0, 4.0, 5.0];
                approx::assert_relative_eq!(actual.as_slice(), expected.as_slice());
            }
        }

        mod and_nonzero_coefficient {
            #[test]
            fn does_not_change_the_original_chromosome() {
                let actual = super::actual(0.5);
                let expected = vec![1.0, 2.0, 3.0, 4.0, 5.0];
                approx::assert_relative_eq!(actual.as_slice(), expected.as_slice());
            }
        }
    }

    mod given_fifty_fifty_chance {
        fn actual(coeff: f32) -> Vec<f32> {
            super::actual(0.5, coeff)
        }

        mod and_zero_coefficient {
            #[test]
            fn does_not_change_the_original_chromosome() {
                let actual = super::actual(0.0);
                let expected = vec![1.0, 2.0, 3.0, 4.0, 5.0];
                approx::assert_relative_eq!(actual.as_slice(), expected.as_slice());
            }
        }

        mod and_nonzero_coefficient {
            #[test]
            fn slightly_changes_the_original_chromosome() {
                let actual = super::actual(0.5);
                let expected = vec![1.0, 1.7756249, 3.0, 4.1596804, 5.0];
                approx::assert_relative_eq!(actual.as_slice(), expected.as_slice());
            }
        }
    }

    mod given_max_chance {
        fn actual(coeff: f32) -> Vec<f32> {
            super::actual(1.0, coeff)
        }

        mod and_zero_coefficient {
            #[test]
            fn does_not_change_the_original_chromosome() {
                let actual = super::actual(0.0);
                let expected = vec![1.0, 2.0, 3.0, 4.0, 5.0];
                approx::assert_relative_eq!(actual.as_slice(), expected.as_slice());
            }
        }

        mod and_nonzero_coefficient {
            #[test]
            fn entirely_changes_the_original_chromosome() {
                let actual = super::actual(0.5);
                let expected = vec![1.4545316, 2.1162078, 2.7756248, 3.9505124, 4.638691];
                approx::assert_relative_eq!(actual.as_slice(), expected.as_slice());
            }
        }
    }
}
