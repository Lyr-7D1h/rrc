use anyhow::{anyhow, Context, Result};
use log::info;
use rapier3d::{
    control::KinematicCharacterController,
    na::{Isometry3, Point3, Quaternion, Translation3, Unit, UnitQuaternion, Vector3},
    parry::utils::Array1,
    prelude::*,
};
use serde::{self, Serialize};
use tokio::sync::mpsc::Receiver;

use crate::{server::Command, specs::Specs};

#[derive(Serialize)]
pub struct Robot {
    #[serde(skip_serializing)]
    rigid_body_set: RigidBodySet,
    rigid_body_handles: Vec<RigidBodyHandle>,
    collider_set: ColliderSet,
    colidder_set_handles: Vec<ColliderHandle>,
    joint_set: MultibodyJointSet,
}

impl Robot {
    pub fn from_specs(specs: Specs) -> Result<Robot> {
        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();
        let mut joint_set = MultibodyJointSet::new();

        let rg_handles: Vec<RigidBodyHandle> = specs
            .links
            .into_iter()
            .map(|link| {
                let rg = RigidBodyBuilder::kinematic_position_based()
                    .lock_translations()
                    .lock_rotations()
                    .build();
                let rg_handle = rigid_body_set.insert(rg);
                let vertices: Vec<Point3<f32>> = link
                    .vertices
                    .chunks(3)
                    .map(|c| match c {
                        &[x, y, z, ..] => Point3::new(x, y, z),
                        _ => panic!(),
                    })
                    .collect();
                let indices = link
                    .indices
                    .chunks(3)
                    .map(|c| match c {
                        &[x, y, z, ..] => [x, y, z],
                        _ => panic!(),
                    })
                    .collect();
                if vertices.len() > 0 {
                    let link_collider = ColliderBuilder::trimesh(vertices, indices)
                        // lets say each link is made of steel
                        .density(7.85)
                        .build();
                    let col_handle = collider_set.insert_with_parent(
                        link_collider,
                        rg_handle,
                        &mut rigid_body_set,
                    );
                }

                rg_handle
            })
            .collect();

        let joint_handles = specs
            .joints
            .into_iter()
            .map(|joint_spec| {
                let rg1 = rg_handles
                    .get(joint_spec.link1.index)
                    .context("invalid index")?
                    .clone();
                let rg2 = rg_handles
                    .get(joint_spec.link1.index)
                    .context("invalid index")?
                    .clone();

                let frame1 = {
                    let [x, y, z] = joint_spec.link1.position;
                    let trans = Translation3::new(x, y, z);
                    let [x, y, z] = joint_spec.link1.direction;
                    let quat = UnitQuaternion::new(Vector3::<f32>::new(x, y, z));
                    Isometry3::from_parts(trans, quat)
                };

                let frame2 = {
                    let [x, y, z] = joint_spec.link2.position;
                    let trans = Translation3::new(x, y, z);
                    let [x, y, z, w] = joint_spec.link2.quaternion;
                    let quat = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
                    Isometry3::from_parts(trans, quat)
                };

                let handle = match joint_spec.r#type.as_str() {
                    "static" => {
                        let joint = FixedJointBuilder::new()
                            .local_frame1(frame1)
                            .local_frame2(frame2);
                        joint_set.insert(rg1, rg2, joint, true)
                    }
                    "rotational" => {
                        println!("{:?}", frame1.rotation.axis());
                        let axis = frame1.rotation.axis().context("invalid axis")?;
                        let joint = RevoluteJointBuilder::new(axis)
                            .local_anchor1(frame1.translation.vector.into())
                            .local_anchor2(frame2.translation.vector.into());
                        joint_set.insert(rg1, rg2, joint, true)
                    }
                    "sliding" => {
                        let axis = frame1.rotation.axis().context("invalid axis")?;
                        let mut joint = PrismaticJoint::new(axis);
                        joint.set_local_anchor1(frame1.translation.vector.into());
                        joint.set_local_anchor2(frame2.translation.vector.into());
                        joint_set.insert(rg1, rg2, joint, true)
                    }
                    _ => return Err(anyhow!("unknown joint type")),
                }
                .context("invalid multibody joint")?;

                Ok(handle)
            })
            .collect::<Result<Vec<MultibodyJointHandle>>>()?;

        // Create the character controller, here with the default configuration.

        Ok(Robot {
            rigid_body_set,
            joint_set,
            collider_set,
        })
    }
}

pub struct Simulation {
    gravity: Vector3<Real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    collider_set: ColliderSet,
    rigid_body_set: RigidBodySet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    robot: Robot,
}

// FIXME: temp hack
const SPECS: &'static str = include_str!("../../robot.json");

impl Simulation {
    pub fn new() -> Simulation {
        // FIXME: temp hack
        let specs: Specs = serde_json::from_str(SPECS).unwrap();

        Simulation {
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            robot: Robot::from_specs(specs).unwrap(),
            collider_set: ColliderSet::new(),
            rigid_body_set: RigidBodySet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
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
        let controller = KinematicCharacterController::default();
        info!("Starting simulation");
        loop {
            if let Ok(cmd) = receiver.try_recv() {
                match cmd {
                    Command::Init { specs } => self.robot = Robot::from_specs(specs),
                    Command::Move { position } => controller.move_shape(
                        dt,                   // The timestep length (can be set to SimulationSettings::dt).
                        &self.rigid_body_set, // The RigidBodySet.
                        &colliders,           // The ColliderSet.
                        &queries,             // The QueryPipeline.
                        self.robot,           // The character’s shape.
                        character_pos,        // The character’s initial position.
                        desired_translation,
                        QueryFilter::default()
                            // Make sure the the character we are trying to move isn’t considered an obstacle.
                            .exclude_rigid_body(character_handle),
                        |_| {}, // We don’t care about events in this example.
                    ),
                }
            }
            self.physics_pipeline.step(
                &self.gravity,
                &self.integration_parameters,
                &mut self.island_manager,
                &mut self.broad_phase,
                &mut self.narrow_phase,
                &mut self.rigid_body_set,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                &mut self.ccd_solver,
                None,
                &(),
                &(),
            );
            cb(self)
        }
    }
}
