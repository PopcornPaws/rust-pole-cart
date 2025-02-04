use super::Individual;
use rand::seq::SliceRandom;
use rand::RngCore;

pub trait SelectionMethod {
    fn select<'a, I>(&self, rng: &mut dyn RngCore, population: &'a [I]) -> &'a I
    where
        I: Individual;
}

pub struct RouletteWheelSelection;

impl RouletteWheelSelection {
    pub fn new() -> Self {
        Self
    }
}

impl SelectionMethod for RouletteWheelSelection {
    fn select<'a, I>(&self, rng: &mut dyn RngCore, population: &'a [I]) -> &'a I
    where
        I: Individual,
    {
        population
            .choose_weighted(rng, |individual| individual.fitness())
            .expect("got an empty population")
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::TestIndividual;

    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng as Cc8;
    use std::collections::BTreeMap;
    use std::iter::FromIterator;

    #[test]
    fn roulette_wheel_selection() {
        let method = RouletteWheelSelection::new();
        let mut rng = Cc8::from_seed(Default::default());
        let mut actual_histogram = BTreeMap::new();

        let population = vec![
            TestIndividual::new_with_fitness(2.0),
            TestIndividual::new_with_fitness(1.0),
            TestIndividual::new_with_fitness(4.0),
            TestIndividual::new_with_fitness(3.0),
        ];

        for _ in 0..1000 {
            let fitness = method.select(&mut rng, &population).fitness() as i32;

            *actual_histogram.entry(fitness).or_insert(0) += 1;
        }

        let expected_histogram = BTreeMap::from_iter(vec![(1, 98), (2, 202), (3, 278), (4, 422)]);

        assert_eq!(actual_histogram, expected_histogram);
    }
}
