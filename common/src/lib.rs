pub use rand::seq::SliceRandom;
pub use rand::{Rng, RngCore};

#[cfg(feature = "dev")]
pub use rand::SeedableRng;
#[cfg(feature = "dev")]
pub use rand_chacha::ChaCha8Rng as Cc8;
