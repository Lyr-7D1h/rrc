use anyhow::{anyhow, Context, Result};
use log::{error, info};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;
use nphysics3d::nalgebra::Vector3;
use nphysics3d::object::{DefaultBodySet, DefaultColliderSet};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};
use serde::{self, Deserialize, Serialize};
use tokio::sync::mpsc::Receiver;

use crate::{server::Command, specs::Specs};

#[derive(Serialize)]
pub struct Robot {
    // #[serde(skip_serializing)]
    // rigid_body_set: RigidBodySet,
    // rigid_body_handles: Vec<RigidBodyHandle>,
    // collider_set: ColliderSet,
    // colidder_set_handles: Vec<ColliderHandle>,
    // joint_set: MultibodyJointSet,
}

impl Robot {
    pub fn from_specs(specs: Specs) -> Result<Robot> {
        Ok(Robot {})
    }

    pub fn joint_values(&self) -> Vec<JointValue> {
        vec![]
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct JointValue {
    index: usize,
    value: f32,
}

pub struct Simulation {
    mechanical_world: DefaultMechanicalWorld<f32>,
    geometrical_world: DefaultGeometricalWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    joint_constraints: DefaultJointConstraintSet<f32>,
    force_generators: DefaultForceGeneratorSet<f32>,
    robot: Robot,
}

// FIXME: temp hack
const SPECS: &'static str = include_str!("../../robot.json");

impl Simulation {
    pub fn new() -> Simulation {
        // FIXME: temp hack
        let specs: Specs = serde_json::from_str(SPECS).unwrap();

        Simulation {
            mechanical_world: DefaultMechanicalWorld::new(Vector3::new(0.0, 0.0, -9.81)),
            geometrical_world: DefaultGeometricalWorld::new(),

            bodies: DefaultBodySet::new(),
            colliders: DefaultColliderSet::new(),
            joint_constraints: DefaultJointConstraintSet::new(),
            force_generators: DefaultForceGeneratorSet::new(),
            robot: Robot::from_specs(specs).unwrap(),
        }
    }

    pub fn robot(&self) -> &Robot {
        &self.robot
    }

    /// Blocking operation that will callback every step
    pub fn run<F>(&mut self, mut receiver: Receiver<Command>, cb: F)
    where
        F: Fn(&mut Self),
    {
        info!("Starting simulation");
        loop {
            if let Ok(cmd) = receiver.try_recv() {
                match cmd {
                    Command::Init { specs } => match Robot::from_specs(specs) {
                        Ok(r) => {
                            info!("creating new robot from specs");
                            self.robot = r;
                        }
                        Err(e) => error!("failed to create robot"),
                    },
                    Command::Move { position } => {}
                }
            }
            self.mechanical_world.step(
                &mut self.geometrical_world,
                &mut self.bodies,
                &mut self.colliders,
                &mut self.joint_constraints,
                &mut self.force_generators,
            );
            cb(self)
        }
    }
}
