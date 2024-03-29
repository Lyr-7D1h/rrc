use std::time::Duration;

use anyhow::Result;
use log::{error, info};

use nalgebra::Vector3;
use tokio::sync::mpsc::Receiver;

use crate::robot::{Robot, State};
use crate::server::Command;

pub struct Simulation {
    robot: Option<Robot>,
}

impl Simulation {
    pub fn new() -> Result<Simulation> {
        Ok(Simulation { robot: None })
    }

    pub fn robot_state(&self) -> Option<&State> {
        if let Some(r) = &self.robot {
            return Some(r.state());
        }
        return None;
    }

    pub fn exec_command(&mut self, command: Command) {
        match command {
            Command::Init {
                urdf,
                limits,
                state,
            } => match Robot::from_specs(urdf, limits, state) {
                Ok(r) => {
                    info!("creating new robot from specs");
                    self.robot = Some(r);
                    // TODO: save to disc
                }
                Err(e) => error!("failed to create robot: {e}"),
            },
            Command::Ikmove { position } => {
                if let Err(e) = self.robot.as_mut().unwrap().move_ik(Vector3::new(
                    position[0],
                    position[1],
                    position[2],
                )) {
                    error!("failed to move robot: {e}");
                }
            }
            Command::Move { state } => {
                if let Err(e) = self.robot.as_mut().unwrap().move_to_state(state) {
                    error!("failed to move robot to state: {e}");
                }
            }
        }
    }

    /// Blocking operation that will callback every step
    pub async fn run<F>(&mut self, mut receiver: Receiver<Command>, cb: F)
    where
        F: Fn(&mut Self),
    {
        info!("Starting simulation");

        // TODO: want to run continously and get the average dt
        // update state every 10ms
        let mut interval = tokio::time::interval(Duration::from_millis(10));

        loop {
            interval.tick().await;

            if let Ok(cmd) = receiver.try_recv() {
                self.exec_command(cmd);
            }

            if let Some(r) = &mut self.robot {
                r.update(10.0)
            }

            cb(self)
        }
    }
}
