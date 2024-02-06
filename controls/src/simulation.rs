use anyhow::Result;
use log::{error, info};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;

use nphysics3d::nalgebra::Vector3;

use nphysics3d::object::{DefaultBodySet, DefaultColliderSet};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

use tokio::sync::mpsc::Receiver;

use crate::robot::{Robot, State};
use crate::server::Command;

pub struct Simulation {
    mechanical_world: DefaultMechanicalWorld<f32>,
    geometrical_world: DefaultGeometricalWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    joint_constraints: DefaultJointConstraintSet<f32>,
    force_generators: DefaultForceGeneratorSet<f32>,
    robot: Option<Robot>,
}

impl Simulation {
    pub fn new() -> Result<Simulation> {
        Ok(Simulation {
            mechanical_world: DefaultMechanicalWorld::new(Vector3::new(0.0, 0.0, 0.0)),
            geometrical_world: DefaultGeometricalWorld::new(),

            joint_constraints: DefaultJointConstraintSet::new(),
            force_generators: DefaultForceGeneratorSet::new(),

            robot: None,
            bodies: DefaultBodySet::new(),
            colliders: DefaultColliderSet::new(),
        })
    }

    pub fn robot_state(&self) -> Option<&State> {
        if let Some(r) = &self.robot {
            return Some(r.state(&self.bodies));
        }
        return None;
    }

    pub fn exec_command(&mut self, command: Command) {
        match command {
            Command::Init { specs } => {
                match Robot::from_specs(&mut self.bodies, &mut self.colliders, specs) {
                    Ok(r) => {
                        info!("creating new robot from specs");
                        println!("{:?}", r.state(&self.bodies));
                        self.robot = Some(r);
                    }
                    Err(e) => error!("failed to create robot: {e}"),
                }
            }
            Command::Ikmove { position } => {
                info!("ikmove of robot to {position:?}");
                if let Err(e) = self.robot.as_mut().unwrap().move_ik(k::Vector3::new(
                    position[0],
                    position[1],
                    position[2],
                )) {
                    error!("failed to move robot: {e}");
                }
            }
            Command::Move { state } => {
                info!("moving robot to {state:?}");
                if let Err(e) = self
                    .robot
                    .as_mut()
                    .unwrap()
                    .move_to_state(&mut self.bodies, state)
                {
                    error!("failed to move robot to state: {e}");
                }
            }
        }
    }

    /// Blocking operation that will callback every step
    pub fn run<F>(&mut self, mut receiver: Receiver<Command>, cb: F)
    where
        F: Fn(&mut Self),
    {
        info!("Starting simulation");
        loop {
            if let Ok(cmd) = receiver.try_recv() {
                self.exec_command(cmd);
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
