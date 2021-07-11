use std::iter::{FromIterator, IntoIterator};
use std::ops::Index;

pub struct Chromosome {
    genes: Vec<f32>,
}

impl Chromosome {
    pub fn len(&self) -> usize {
        self.genes.len()
    }

    pub fn iter(&self) -> impl Iterator<Item = &f32> {
        self.genes.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut f32> {
        self.genes.iter_mut()
    }
}

impl Index<usize> for Chromosome {
    type Output = f32;
    fn index(&self, index: usize) -> &Self::Output {
        &self.genes[index]
    }
}

impl FromIterator<f32> for Chromosome {
    fn from_iter<T: IntoIterator<Item = f32>>(iter: T) -> Self {
        Self {
            genes: iter.into_iter().collect(),
        }
    }
}

impl IntoIterator for Chromosome {
    type Item = f32;
    type IntoIter = impl Iterator<Item = f32>;

    fn into_iter(self) -> Self::IntoIter {
        self.genes.into_iter()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn chromosome_methods() {
        let chromosome = Chromosome::from_iter(vec![1.0, 2.0, 3.0, 4.0]);
        assert_eq!(chromosome.len(), 4);

        assert!((chromosome[0] - 1.0).abs() < f32::EPSILON);
        assert!((chromosome[1] - 2.0).abs() < f32::EPSILON);
        assert!((chromosome[2] - 3.0).abs() < f32::EPSILON);
        assert!((chromosome[3] - 4.0).abs() < f32::EPSILON);

        let genes = chromosome.into_iter().collect::<Vec<f32>>();

        assert!((genes[0] - 1.0).abs() < f32::EPSILON);
        assert!((genes[1] - 2.0).abs() < f32::EPSILON);
        assert!((genes[2] - 3.0).abs() < f32::EPSILON);
        assert!((genes[3] - 4.0).abs() < f32::EPSILON);

        let mut chromosome = Chromosome::from_iter(vec![1.0, 2.0, 3.0, 4.0]);

        chromosome.iter_mut().for_each(|x| *x += 1.0);

        assert!((chromosome[0] - 2.0).abs() < f32::EPSILON);
        assert!((chromosome[1] - 3.0).abs() < f32::EPSILON);
        assert!((chromosome[2] - 4.0).abs() < f32::EPSILON);
        assert!((chromosome[3] - 5.0).abs() < f32::EPSILON);

        let new = chromosome.iter().map(|x| x + 2.0).collect::<Vec<f32>>();

        assert!((new[0] - 4.0).abs() < f32::EPSILON);
        assert!((new[1] - 5.0).abs() < f32::EPSILON);
        assert!((new[2] - 6.0).abs() < f32::EPSILON);
        assert!((new[3] - 7.0).abs() < f32::EPSILON);
    }
}
