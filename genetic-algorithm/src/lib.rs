#![feature(min_type_alias_impl_trait)]

mod chromosome;
mod crossover;
mod individual;
mod mutation;
mod selection;

use chromosome::Chromosome;
use individual::Individual;

pub use crossover::{CrossoverMethod, UniformCrossover};
pub use mutation::{MutationMethod, GaussianMutation};
pub use selection::{SelectionMethod, RouletteWheelSelection};

use rand::RngCore;

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
        population
            .iter()
            .map(|individual| {
                let parent_a = self.selection_method.select(rng, population).chromosome();
                let parent_b = self.selection_method.select(rng, population).chromosome();

                let mut child = self.crossover_method.crossover(rng, &parent_a, &parent_b);

                self.mutation_method.mutate(rng, &mut child);
                // TODO convert Chromosome back into Individual
                todo!()
            })
            .collect()
    }
}
