#![feature(min_type_alias_impl_trait)]

mod chromosome;
mod crossover;
mod individual;
mod mutation;
mod selection;

use chromosome::Chromosome;
use individual::Individual;

// export traits as well, in case someone wants to implement their own
// algorithms externally
pub use crossover::{CrossoverMethod, UniformCrossover};
pub use mutation::{GaussianMutation, MutationMethod};
pub use selection::{RouletteWheelSelection, SelectionMethod};

use common::RngCore;

pub struct GeneticAlgorithm<C, M, S> {
    crossover_method: C,
    mutation_method: M,
    selection_method: S,
}

impl<C, M, S> GeneticAlgorithm<C, M, S>
where
    C: CrossoverMethod,
    M: MutationMethod,
    S: SelectionMethod,
{
    pub fn new(crossover_method: C, mutation_method: M, selection_method: S) -> Self {
        Self {
            crossover_method,
            mutation_method,
            selection_method,
        }
    }

    pub fn evolve<I>(&self, rng: &mut dyn RngCore, population: &[I]) -> Vec<I>
    where
        I: Individual,
    {
        assert!(!population.is_empty());
        (0..population.len())
            .map(|_| {
                let parent_a = self.selection_method.select(rng, population).chromosome();
                let parent_b = self.selection_method.select(rng, population).chromosome();

                let mut child = self.crossover_method.crossover(rng, &parent_a, &parent_b);

                self.mutation_method.mutate(rng, &mut child);
                I::create(child)
            })
            .collect()
    }
}

#[cfg(test)]
#[derive(Debug, PartialEq)]
enum TestIndividual {
    WithChromosome { chromosome: Chromosome },
    WithFitness { fitness: f32 },
}

#[cfg(test)]
impl TestIndividual {
    fn new_with_fitness(fitness: f32) -> Self {
        Self::WithFitness { fitness }
    }
}

#[cfg(test)]
impl Individual for TestIndividual {
    fn fitness(&self) -> f32 {
        match self {
            Self::WithFitness { fitness } => *fitness,
            Self::WithChromosome { chromosome } => chromosome.iter().sum(),
        }
    }

    fn create(chromosome: Chromosome) -> Self {
        Self::WithChromosome { chromosome }
    }

    fn chromosome(&self) -> &Chromosome {
        match self {
            Self::WithChromosome { chromosome } => &chromosome,
            _ => panic!("not supported on this variant"),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use common::{Cc8, SeedableRng};

    fn individual(genes: &[f32]) -> TestIndividual {
        let chromosome = genes.iter().cloned().collect();
        TestIndividual::create(chromosome)
    }

    #[test]
    fn evolution() {
        let mut rng = Cc8::from_seed(Default::default());
        let ga = GeneticAlgorithm::new(
            UniformCrossover::new(),
            GaussianMutation::new(0.5, 0.5),
            RouletteWheelSelection::new(),
        );

        let mut population = vec![
            individual(&[0.0, 0.0, 0.0]), // fitness = 0.0
            individual(&[1.0, 1.0, 1.0]), // fitness = 3.0
            individual(&[1.0, 2.0, 1.0]), // fitness = 4.0
            individual(&[1.0, 2.0, 4.0]), // fitness = 7.0
        ];

        for _ in 0..10 {
            population = ga.evolve(&mut rng, &population);
        }

        let expected_population = vec![
            individual(&[0.44769490, 2.0648358, 4.3058133]),
            individual(&[1.21268670, 1.5538777, 2.8869110]),
            individual(&[1.06176780, 2.2657390, 4.4287640]),
            individual(&[0.95909685, 2.4618788, 4.0247330]),
        ];

        assert_eq!(population, expected_population);
    }
}
