mod cart;
use cart::Cart;

use genetic_algorithm::GeneticAlgorithm;

pub struct Simulation<C, M, S> {
    carts: Vec<Cart>,
    height_goal: f32,
    genetic_algorithm: GeneticAlgorithm<C, M, S>,
}
