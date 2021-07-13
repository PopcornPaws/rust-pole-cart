mod cart;
use cart::Cart;

use genetic_algorithm::GeneticAlgorithm;

pub struct Simulation<S, C, M, I> {
    genetic_algorithm: GeneticAlgorithm<S, C, M>,
    individuals: Vec<I>,
}
