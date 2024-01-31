use log::info;
use rapier3d::{control::KinematicCharacterController, na::Vector3, prelude::*};
use serde::{self, Serialize};

pub struct Specs {}

#[derive(Serialize)]
pub struct Robot {
    #[serde(skip_serializing)]
    controller: KinematicCharacterController,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    joint_set: MultibodyJointSet,
}

impl Robot {
    pub fn new() -> Robot {
        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();
        let mut joint_set = MultibodyJointSet::new();

        let base = RigidBodyBuilder::kinematic_position_based()
            .lock_translations()
            .lock_rotations()
            .build();
        let base = rigid_body_set.insert(base);

        let link = RigidBodyBuilder::kinematic_position_based()
            .lock_translations()
            .lock_rotations()
            .build();
        let link = rigid_body_set.insert(link);

        let link_collider = ColliderBuilder::trimesh(
            vec![point![1.0, 2.0, 3.0]],
            vec![
                [0, 0, 0], // [0, 2, 1],
                           // [2, 3, 1],
                           // [4, 6, 5],
                           // [6, 7, 5],
                           // [8, 10, 9],
                           // [10, 11, 9],
                           // [12, 14, 13],
                           // [14, 15, 13],
                           // [16, 18, 17],
                           // [18, 19, 17],
                           // [20, 22, 21],
                           // [22, 23, 21],
            ],
        )
        // lets say each link is made of steel
        .density(7.85)
        .build();
        collider_set.insert_with_parent(link_collider, link, &mut rigid_body_set);

        // add a joint between base and arm
        let joint = RevoluteJointBuilder::new(Vector::z_axis())
            .local_anchor1(point![0.0, 0.0, 1.0])
            .local_anchor2(point![0.0, 0.0, -3.0]);
        joint_set.insert(base, link, joint, true);

        // Create the character controller, here with the default configuration.
        let controller = KinematicCharacterController::default();

        Robot {
            controller,
            rigid_body_set,
            joint_set,
            collider_set,
        }
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

impl Simulation {
    pub fn new() -> Simulation {
        Simulation {
            gravity: vector![0.0, -9.81, 0.0],
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            robot: Robot::new(),
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
    pub fn run<F>(&mut self, cb: F)
    where
        F: Fn(&mut Self),
    {
        info!("Starting simulation");
        loop {
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
            // let corrected_movement = character_controller.move_shape(
            //     dt,              // The timestep length (can be set to SimulationSettings::dt).
            //     &bodies,         // The RigidBodySet.
            //     &colliders,      // The ColliderSet.
            //     &queries,        // The QueryPipeline.
            //     character_shape, // The character’s shape.
            //     character_pos,   // The character’s initial position.
            //     desired_translation,
            //     QueryFilter::default()
            //         // Make sure the the character we are trying to move isn’t considered an obstacle.
            //         .exclude_rigid_body(character_handle),
            //     |_| {}, // We don’t care about events in this example.
            // );
        }
    }
}
