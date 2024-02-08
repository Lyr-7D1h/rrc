use std::{any::TypeId, borrow::Borrow};

use anyhow::{anyhow, Context, Result};

use k::{
    nalgebra::ComplexField, Chain, Constraints, InverseKinematicsSolver, SerialChain, Translation3,
};

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

pub type State = Vec<f32>;

fn pose_to_isometry(pose: &urdf_rs::Pose) -> Isometry<f32> {
    let [x, y, z] = pose.xyz.map(|x| x as f32);
    let trans = Translation::new(x, y, z);
    let [r, p, y] = pose.rpy.map(|x| x as f32);
    let quat = UnitQuaternion::from_euler_angles(r, p, y);
    Isometry::from_parts(trans, quat)
}

pub struct Robot {
    velocities: Vec<f32>,
    /// joint values (dofs)
    state: Vec<f32>,
    desired_state: Vec<f32>,
    urdf: urdf_rs::Robot,
    chain: k::Chain<f32>,
    limits: Vec<Limits>,
}

impl Robot {
    pub fn from_disk() -> Robot {
        // TODO: get robot specs from disk
        todo!()
    }

    pub fn from_specs(urdf: urdf_rs::Robot, limits: Vec<Limits>) -> Result<Robot> {
        let chain = k::Chain::from(&urdf);
        let state: State = (0..chain.dof()).map(|_| 0.0).collect();
        chain.set_joint_positions(&state)?;
        chain.update_transforms();

        Ok(Robot {
            velocities: state.clone(),
            desired_state: state.clone(),
            state,
            chain,
            urdf,
            limits,
        })
    }

    /// Move robot using inverse kinematics solver
    pub fn move_ik(&mut self, position: k::Vector3<f32>) -> Result<()> {
        println!("{}", self.chain);
        // FIXME: assumes end effector name
        let end_effector = self
            .chain
            .find("gripper")
            .context("could not find end effector")?;

        self.chain.update_transforms();
        let mut target = end_effector
            .world_transform()
            .context("failed to get world transform")?;
        target.translation = Translation3::new(position.x, position.y, position.z);

        let solver = k::JacobianIkSolver::default();

        let serial = SerialChain::from_end(end_effector);
        // FIXME: allow for different constraints
        let constraints = Constraints {
            position_x: false,
            position_y: false,
            position_z: true,
            rotation_x: true,
            rotation_y: true,
            rotation_z: true,
            ignored_joint_names: vec![],
        };
        solver.solve_with_constraints(&serial, &target, &constraints)?;
        self.chain.update_transforms();
        let solved_pose = end_effector.world_transform().unwrap();
        for (i, trans) in serial.update_transforms().iter().enumerate() {}
        println!("sol: {solved_pose}");

        return Ok(());
    }

    pub fn move_to_state(&mut self, desired_state: State) -> Result<()> {
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

    pub fn update(&mut self, dt_ms: f32) {
        for i in 0..self.state.len() {
            // currently velocity in mm/ms
            let v = self.velocities[i];
            self.state[i] += v * (dt_ms / 1000.0);

            // remove velocity if target reached
            if self.state[i] == self.desired_state[i] {
                self.velocities[i] = 0.0;
                continue;
            }

            //  mm/s
            let max_v = self.limits[i].velocity;
            // mm
            let e = self.desired_state[i] - self.state[i];
            // optimal velocity for time frame mm/s
            let v_o = e / (dt_ms / 1000.0);

            if v_o.abs() > max_v {
                self.velocities[i] = e.signum() * max_v;
            } else {
                self.velocities[i] = v_o;
            }
        }
    }
}
