use anyhow::{anyhow, Result};

use log::{info, warn};
use nalgebra::{vector, Vector3};

use crate::{ik::Fabrik, server::Limits};

pub type State = Vec<f64>;

pub struct Robot {
    velocities: Vec<f64>,
    accelerations: Vec<f64>,
    /// joint values (dofs)
    state: State,
    /// desired joint values (dofs)
    desired_state: State,
    _urdf: urdf_rs::Robot,
    fabrik: Fabrik,
    limits: Vec<Limits>,
}

impl Robot {
    pub fn from_disk() -> Robot {
        // TODO: get robot specs from disk
        todo!()
    }

    pub fn from_specs(
        urdf: urdf_rs::Robot,
        limits: Vec<Limits>,
        initial_state: State,
    ) -> Result<Robot> {
        // assume that robot is not moving now
        let velocities: State = (0..initial_state.len()).map(|_| 0.0).collect();
        let accelerations: State = velocities.clone();

        // TODO: get joint positions from spec
        let joint_positions = vec![
            vector![0.0, 0.0],
            vector![0.0, 725.0],
            vector![0.0, 1325.0],
            // end effector
            vector![0.0, 1525.0],
        ];
        let constraints = limits.iter().map(|l| (l.min, l.max)).collect();
        let fabrik = Fabrik::new(joint_positions, constraints, 0.000000001, 1000)?;

        Ok(Robot {
            velocities,
            accelerations,
            desired_state: initial_state.clone(),
            state: initial_state,
            _urdf: urdf,
            fabrik,
            limits,
        })
    }

    // TODO: add support for different joints and 3d space solutions
    /// Move robot using FABRIK as inverse kinematics solver on a planar field
    pub fn move_ik(&mut self, position: Vector3<f64>) -> Result<()> {
        info!("solving ik to move to {position:?}");

        let target = vector![position.x, position.y];

        if self.fabrik.solvable(position.xy()) == false {
            warn!("target is out of reach will try to reach");
        }

        let iterations = self.fabrik.move_to(target, true)?;
        info!("ik solved within {iterations} iterations");

        let angles = self.fabrik.angles_deg();

        let lift = 1525.0 - position.z;

        let desired_state = vec![angles[0], lift, angles[1], angles[2], self.desired_state[4]];

        self.move_to_state(desired_state)?;

        Ok(())
    }

    pub fn move_to_state(&mut self, desired_state: State) -> Result<()> {
        // check if state is within bounds
        for (i, v) in desired_state.iter().enumerate() {
            let l = &self.limits[i];
            if *v < l.min || *v > l.max {
                return Err(anyhow!(
                    "joint {i}'s desired state is not within bound of {} < {v} < {}",
                    l.min,
                    l.max
                ));
            }
        }

        info!("moving robot to {desired_state:?}");

        self.desired_state = desired_state;

        return Ok(());
    }

    pub fn state(&self) -> &State {
        return &self.state;
    }

    /// update a joint to go from a to b using trapezoidal motion planning given max acceleration
    /// and velocity
    pub fn update(&mut self, dt_ms: f64) {
        // time in seconds
        let t = dt_ms / 1000.0;

        for i in 0..self.state.len() {
            let a = self.accelerations[i] as f64;
            // update velocity
            self.velocities[i] += a * t;
            let v = self.velocities[i];
            // upate position
            self.state[i] += v * t;

            // distance left to go
            let d = self.desired_state[i] - self.state[i];
            let sign = d.signum();
            let d = d.abs();

            let v_sign = v.signum();
            let v = v.abs();

            // if not moving and at desired destination skip
            if d < 0.00000001 && v.abs() < 0.00000001 {
                // if within a fair amount of rounding error reset everything
                self.accelerations[i] = 0.0;
                self.velocities[i] = 0.0;
                self.desired_state[i] = self.state[i];
                continue;
            }

            //  {unit}/s
            let a_max = self.limits[i].acceleration;
            let v_max = self.limits[i].velocity;

            let mut a_o = if v == v_max {
                0.0
            // v of next time sample is above v_max
            } else if v + a_max * t > v_max {
                // acceleration needed to go to v_max next step
                sign * ((v_max - v) / t)
            } else {
                sign * a_max
            };

            let v_expected = v + a_o.abs() * t;
            let v_break = -a_max * t + (2.0 * d * a_max).sqrt();
            if v_break < v_expected {
                // next time step is beyond breaking point

                // acceleration needed to go to zero while compensating for undershooting distance
                // a slope that will create a same distance as the slope for breaking point slope
                a_o = v_sign * -(v.powi(2)) / (2.0 * d);
            }

            // ensure limit to direcion of a
            if a_o.abs() > a_max {
                a_o = a_o.signum() * a_max;
            }

            self.accelerations[i] = a_o;
        }
    }
}
