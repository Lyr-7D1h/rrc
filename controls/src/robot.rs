use std::{any::TypeId, borrow::Borrow};

use anyhow::{anyhow, Context, Result};

// use k::{nalgebra::ComplexField, Constraints, InverseKinematicsSolver, SerialChain, Translation3};

use log::info;
use nphysics3d::{
    joint::{FixedJoint, Joint, PrismaticJoint, RevoluteJoint},
    math::{AngularVector, Isometry, Translation},
    nalgebra::{Unit, UnitQuaternion, Vector3},
    ncollide3d::shape::{Cuboid, Shape, ShapeHandle},
    object::{
        BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodySet, DefaultColliderSet,
        Multibody, MultibodyDesc,
    },
};

use crate::server::Limits;

pub type State = Vec<f64>;

fn pose_to_isometry(pose: &urdf_rs::Pose) -> Isometry<f32> {
    let [x, y, z] = pose.xyz.map(|x| x as f32);
    let trans = Translation::new(x, y, z);
    let [r, p, y] = pose.rpy.map(|x| x as f32);
    let quat = UnitQuaternion::from_euler_angles(r, p, y);
    Isometry::from_parts(trans, quat)
}

pub struct Robot {
    velocities: Vec<f64>,
    accelerations: Vec<f64>,
    /// joint values (dofs)
    state: State,
    desired_state: State,
    urdf: urdf_rs::Robot,
    chain: k::Chain<f64>,
    limits: Vec<Limits>,
    pids: Vec<Pid>,
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
        let chain = k::Chain::from(&urdf);
        // assume that robot is not moving now
        let velocities: State = (0..chain.dof()).map(|_| 0.0).collect();
        let accelerations: State = velocities.clone();
        let pids = (0..chain.dof()).map(|_| Pid::new(1.0, 0.1, 0.01)).collect();
        chain.set_joint_positions(&initial_state)?;
        chain.update_transforms();

        Ok(Robot {
            velocities,
            accelerations,
            desired_state: initial_state.clone(),
            state: initial_state,
            pids,
            chain,
            urdf,
            limits,
        })
    }

    /// Move robot using inverse kinematics solver
    pub fn move_ik(&mut self, position: k::Vector3<f32>) -> Result<()> {
        todo!()
        // println!("{}", self.chain);
        // // FIXME: assumes end effector name
        // let end_effector = self
        //     .chain
        //     .find("gripper")
        //     .context("could not find end effector")?;
        //
        // self.chain.update_transforms();
        // let mut target = end_effector
        //     .world_transform()
        //     .context("failed to get world transform")?;
        // target.translation = Translation3::new(position.x, position.y, position.z);
        //
        // let solver = k::JacobianIkSolver::default();
        //
        // let serial = SerialChain::from_end(end_effector);
        // // FIXME: allow for different constraints
        // let constraints = Constraints {
        //     position_x: false,
        //     position_y: false,
        //     position_z: true,
        //     rotation_x: true,
        //     rotation_y: true,
        //     rotation_z: true,
        //     ignored_joint_names: vec![],
        // };
        // solver.solve_with_constraints(&serial, &target, &constraints)?;
        // self.chain.update_transforms();
        // let solved_pose = end_effector.world_transform().unwrap();
        // for (i, trans) in serial.update_transforms().iter().enumerate() {}
        // println!("sol: {solved_pose}");
        //
        // return Ok(());
    }

    pub fn move_to_state(&mut self, desired_state: State) -> Result<()> {
        // check if state is within bounds
        for (i, v) in desired_state.iter().enumerate() {
            let l = &self.limits[i];
            if *v < l.min || *v > l.max {
                return Err(anyhow!(
                    "desired state is not within bound of {} < {v} < {}",
                    l.min,
                    l.max
                ));
            }
        }

        self.desired_state = desired_state;
        // self.chain.set_joint_positions(&self.state)?;
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
            if d < 0.000001 && v.abs() < 0.000001 {
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
            // println!(
            //     "velocity={} acc={} dist={} {} {}",
            //     v_sign * v,
            //     // v,
            //     self.accelerations[i],
            //     sign * d,
            //     v_break,
            //     v_expected
            // );
        }
    }
}

struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    prev_error: f32,
}
impl Pid {
    fn new(kp: f32, ki: f32, kd: f32) -> Pid {
        Pid {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
        }
    }

    fn update(&mut self, s_t: f32, p_t: f32, dt: f32) -> f32 {
        let e = s_t - p_t;

        let p = self.kp * e;

        self.integral += e * dt;
        let i = self.ki * self.integral;

        let mut d = (e - self.prev_error) / dt;
        d *= self.kd;

        self.prev_error = e;
        return p + i + d;
    }
}
