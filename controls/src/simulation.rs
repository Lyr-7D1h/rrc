use anyhow::Result;
use log::{error, info};
use nphysics3d::force_generator::DefaultForceGeneratorSet;
use nphysics3d::joint::DefaultJointConstraintSet;

use nphysics3d::nalgebra::Vector3;

use nphysics3d::object::{DefaultBodySet, DefaultColliderSet};
use nphysics3d::world::{DefaultGeometricalWorld, DefaultMechanicalWorld};

use tokio::sync::mpsc::Receiver;

use crate::robot::Robot;
use crate::server::Command;

pub struct Simulation {
    mechanical_world: DefaultMechanicalWorld<f32>,
    geometrical_world: DefaultGeometricalWorld<f32>,
    bodies: DefaultBodySet<f32>,
    colliders: DefaultColliderSet<f32>,
    joint_constraints: DefaultJointConstraintSet<f32>,
    force_generators: DefaultForceGeneratorSet<f32>,
    robot: Robot,
}

impl Simulation {
    pub fn new() -> Result<Simulation> {
        let mut bodies = DefaultBodySet::new();
        let mut colliders = DefaultColliderSet::new();

        let robot = Robot::default(&mut bodies, &mut colliders);

        Ok(Simulation {
            mechanical_world: DefaultMechanicalWorld::new(Vector3::new(0.0, 0.0, -9.81)),
            geometrical_world: DefaultGeometricalWorld::new(),

            joint_constraints: DefaultJointConstraintSet::new(),
            force_generators: DefaultForceGeneratorSet::new(),

            robot,
            bodies,
            colliders,
        })
    }

    pub fn robot(&self) -> &Robot {
        &self.robot
    }

    pub fn exec_command(&mut self, command: Command) {
        match command {
            Command::Init { specs } => {
                match Robot::from_specs(&mut self.bodies, &mut self.colliders, specs) {
                    Ok(r) => {
                        info!("creating new robot from specs");
                        self.robot = r;
                    }
                    Err(e) => error!("failed to create robot: {e}"),
                }
            }
            Command::Move { position } => {
                println!("{position:?}");
                if let Err(e) =
                    self.robot
                        .move_ik(k::Vector3::new(position[0], position[1], position[2]))
                {
                    error!("failed to move robot: {e}");
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
