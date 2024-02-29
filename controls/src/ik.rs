use anyhow::{anyhow, bail, Result};
use nalgebra::Vector2;

pub struct Fabrik {
    joints: Vec<Vector2<f64>>,
    constraints: Vec<(f64, f64)>,
    tolerance: f64,
    iteration_limit: usize,
    lengths: Vec<f64>,
    max_length: f64,
}

impl Fabrik {
    pub fn new(
        joint_positions: Vec<Vector2<f64>>,
        constraints: Vec<(f64, f64)>,
        tolerance: f64,
        iteration_limit: usize,
    ) -> Result<Fabrik> {
        if tolerance <= 0.0 {
            return Err(anyhow!("tolerance must be greater than 0"));
        }

        let lengths: Vec<f64> = (1..joint_positions.len())
            .map(|i| (joint_positions[i - 1] - joint_positions[i]).magnitude())
            .collect();

        if lengths.iter().any(|l| *l <= 0.0) {
            return Err(anyhow!("link lengths must be greater than 0"));
        }

        let max_length = lengths.iter().sum();

        Ok(Fabrik {
            joints: joint_positions,
            constraints,
            tolerance,
            iteration_limit,
            lengths,
            max_length,
        })
    }

    pub fn solvable(&self, target: Vector2<f64>) -> bool {
        return self.max_length >= target.magnitude();
    }

    pub fn move_to(&mut self, target: Vector2<f64>, try_to_reach: bool) -> Result<usize> {
        if !self.solvable(target) {
            if !try_to_reach {
                bail!("target not in range");
            }

            return self.iterate(target.normalize().scale(self.max_length));
        }

        self.iterate(target)
    }

    pub fn angles(&self) -> Vec<f64> {
        // TODO: get angle relative to each transform of joint
        // HACK: getting angle relative to y axis due to robot being defined towards y+
        let mut angles = vec![self.joints[1].x.atan2(self.joints[1].y)];

        let mut sum = angles[0];
        for i in 2..self.joints.len() {
            let v = self.joints[i] - self.joints[i - 1];
            let abs_angle = v.x.atan2(v.y);
            angles.push(abs_angle - sum);
            sum += abs_angle;
        }

        angles
    }

    /// Get the angles in degrees
    pub fn angles_deg(&self) -> Vec<f64> {
        self.angles().iter().map(|a| a.to_degrees()).collect()
    }

    fn iterate(&mut self, target: Vector2<f64>) -> Result<usize> {
        // TODO: check if joints are in a straight line if so add a slight angle http://www.andreasaristidou.com/publications/papers/FABRIK.pdf#%5B%7B%22num%22%3A183%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22FitR%22%7D%2C0%2C467%2C545%2C467%5D
        let mut iteration: usize = 0;
        let initial_position: Vector2<f64> = self.joints[0];
        let last = self.joints.len() - 1;

        while (self.joints[last] - target).magnitude() > self.tolerance {
            if iteration > self.iteration_limit {
                bail!(
                    "could not converge under tolerance with error of {}",
                    (self.joints[last] - target).magnitude()
                );
            }

            // TODO: add constraints
            // FIXME: does not converge when all joints are in a straight line and your target is
            // also on this line

            self.joints[last] = target;
            for i in (0..last).rev() {
                let (prev, current) = (self.joints[i + 1], self.joints[i]);
                let d = (current - prev).normalize().scale(self.lengths[i]);
                self.joints[i] = prev + d;
            }

            self.joints[0] = initial_position;
            for i in 1..=last {
                let (prev, current) = (self.joints[i - 1], self.joints[i]);
                let d = (current - prev).normalize().scale(self.lengths[i - 1]);
                self.joints[i] = prev + d;
            }
            iteration += 1;
        }

        Ok(iteration)
    }
}
