use std::{any::TypeId, borrow::Borrow};

use anyhow::{anyhow, Context, Result};

use k::{Chain, Constraints, InverseKinematicsSolver, SerialChain, Translation3};

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

pub type State = Vec<f32>;

fn pose_to_isometry(pose: &urdf_rs::Pose) -> Isometry<f32> {
    let [x, y, z] = pose.xyz.map(|x| x as f32);
    let trans = Translation::new(x, y, z);
    let [r, p, y] = pose.rpy.map(|x| x as f32);
    let quat = UnitQuaternion::from_euler_angles(r, p, y);
    Isometry::from_parts(trans, quat)
}

fn to_phys_joint(joint_spec: &urdf_rs::Joint) -> Result<Box<dyn Joint<f32>>> {
    let phys_joint: Box<dyn Joint<f32>> = match joint_spec.joint_type {
        urdf_rs::JointType::Revolute => {
            let [x, y, z] = joint_spec.axis.xyz.map(|x| x as f32);
            let axis = Unit::new_normalize(AngularVector::new(x, y, z));

            let mut joint_phys = RevoluteJoint::new(axis, 0.0);

            // TODO: add effort and damping effects
            let limits = &joint_spec.limit;
            joint_phys.enable_angular_motor();
            joint_phys.enable_min_angle(limits.lower as f32);
            joint_phys.enable_max_angle(limits.upper as f32);
            joint_phys.set_max_angular_motor_velocity(limits.velocity as f32);
            joint_phys.set_max_angular_motor_torque(100.0);
            Box::new(joint_phys)
        }
        urdf_rs::JointType::Prismatic => {
            let [x, y, z] = joint_spec.axis.xyz.map(|x| x as f32);
            let axis = Unit::new_normalize(Vector3::new(x, y, z));

            let mut joint_phys = PrismaticJoint::new(axis, 0.0);

            // TODO: add effort and damping effects
            let limits = &joint_spec.limit;
            joint_phys.enable_linear_motor();
            joint_phys.enable_min_offset(limits.lower as f32);
            joint_phys.enable_max_offset(limits.upper as f32);
            joint_phys.set_max_linear_motor_velocity(limits.velocity as f32);
            joint_phys.set_max_linear_motor_force(100.0);

            Box::new(joint_phys)
        }
        urdf_rs::JointType::Fixed => {
            let pos = pose_to_isometry(&joint_spec.origin);
            Box::new(FixedJoint::new(pos))
        }
        _ => return Err(anyhow!("unsupported joint type")),
    };
    return Ok(phys_joint);
}

fn to_phys_shape_handle(geometry: &urdf_rs::Geometry) -> Result<ShapeHandle<f32>> {
    let handle = match geometry {
        urdf_rs::Geometry::Box { size } => ShapeHandle::new(Cuboid::new(Vector3::new(
            size[0] as f32,
            size[1] as f32,
            size[2] as f32,
        ))),
        _ => return Err(anyhow!("unsupported geometry")),
    };
    return Ok(handle);
}

pub struct Robot {
    // multibody: DefaultBodyHandle,
    state: Vec<f32>,
    specs: urdf_rs::Robot,
    chain: k::Chain<f32>,
    serial_chain: k::SerialChain<f32>,
}

impl Robot {
    // TODO: get robot specs from disk
    pub fn default(
        _bodies: &mut DefaultBodySet<f32>,
        _colliders: &mut DefaultColliderSet<f32>,
    ) -> Robot {
        todo!()
    }

    pub fn from_specs(
        bodies: &mut DefaultBodySet<f32>,
        colliders: &mut DefaultColliderSet<f32>,
        specs: urdf_rs::Robot,
    ) -> Result<Robot> {
        let chain = k::Chain::from(&specs);
        let state: State = (0..chain.dof()).map(|_| 0.0).collect();
        chain.set_joint_positions(&state)?;
        chain.update_transforms();

        // // FIXME: assumes first one is the base
        // let joint_spec = specs.joints.get(0).context("need more than 1 joint")?;
        // let joint = to_phys_joint(&joint_spec)?;
        // let parent_isometry = pose_to_isometry(&joint_spec.origin);
        // let body_pos = &specs
        //     .links
        //     .iter()
        //     .find(|l| l.name == joint_spec.child.link)
        //     .context("expected child")?
        //     .collision[0]
        //     .origin;
        // let body_isometry = pose_to_isometry(body_pos);
        // let mut multibody_desc: MultibodyDesc<f32> = MultibodyDesc::from_boxed_joint(joint)
        //     .mass(200.0)
        //     // joint position wrt parent
        //     .parent_shift(parent_isometry.translation.vector)
        //     // child position wrt joint
        //     .body_shift(body_isometry.translation.vector);
        //
        // // FIXME: assumes all joints are in order and is in a chain
        // let mut last = &mut multibody_desc;
        // for joint in specs.joints.iter().skip(1) {
        //     let phys_joint = to_phys_joint(joint)?;
        //     let parent_isometry = pose_to_isometry(&joint_spec.origin);
        //     let body_pos = &specs
        //         .links
        //         .iter()
        //         .find(|l| l.name == joint_spec.child.link)
        //         .context("expected child")?
        //         .collision[0]
        //         .origin;
        //     let body_isometry = pose_to_isometry(body_pos);
        //     let desc = MultibodyDesc::from_boxed_joint(phys_joint)
        //         .mass(10.0)
        //         // joint position wrt parent
        //         .parent_shift(parent_isometry.translation.vector)
        //         // child position wrt joint
        //         .body_shift(body_isometry.translation.vector);
        //     last = last.add_child_desc(desc).set_name(joint.name.clone());
        // }
        //
        // let multibody = multibody_desc.build();
        // let multibody = bodies.insert(multibody);

        // FIXME: assumes first one does not have collision
        // for i in 1..specs.links.len() {
        //     let link = &specs.links[i];
        //
        //     if let Some(collision) = link.collision.get(0) {
        //         let handle = to_phys_shape_handle(&collision.geometry)?;
        //         let collider = ColliderDesc::new(handle)
        //             // lets say each link is made of steel
        //             // .density(7.85)
        //             .build(BodyPartHandle(multibody, i - 1));
        //         colliders.insert(collider);
        //     }
        // }

        Ok(Robot {
            state,
            // TODO: allow for non serial chains
            serial_chain: SerialChain::try_new(chain.clone())
                .context("urdf robot is not in series")?,
            // multibody,
            chain,
            specs,
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

    pub fn move_to_state(
        &mut self,
        bodies: &mut DefaultBodySet<f32>,
        desired_state: State,
    ) -> Result<()> {
        self.state = desired_state;
        self.chain.set_joint_positions(&self.state)?;
        // let multibody = bodies
        //     .multibody_mut(self.multibody)
        //     .expect("failed to get robot multibody");
        //
        // for (i, link) in multibody.links_mut().enumerate() {
        //     let joint = link.joint_mut();
        //
        //     if let Some(joint) = joint.downcast_mut::<PrismaticJoint<f32>>() {
        //         println!("{joint:?}");
        //         let value = desired_state
        //             .get(i)
        //             .context("desired_state is missing joint values")?;
        //         // joint.apply_displacement();
        //         joint.set_desired_linear_motor_velocity(2.0);
        //     } else if let Some(joint) = joint.downcast_mut::<RevoluteJoint<f32>>() {
        //         let value = desired_state
        //             .get(i)
        //             .context("desired_state is missing joint values")?;
        //         joint.set_desired_angular_motor_velocity(0.5);
        //     }
        // }

        return Ok(());
    }

    pub fn state(&self, bodies: &DefaultBodySet<f32>) -> &State {
        return &self.state;
        // let multibody = bodies
        //     .multibody(self.multibody)
        //     .expect("failed to get robot multibody");
        //
        // let state = multibody
        //     .links()
        //     .filter_map(|l| {
        //         let joint = l.joint();
        //
        //         if let Some(joint) = joint.downcast_ref::<PrismaticJoint<f32>>() {
        //             return Some(joint.offset());
        //         }
        //         if let Some(joint) = joint.downcast_ref::<RevoluteJoint<f32>>() {
        //             return Some(joint.angle());
        //         }
        //         // FIXME only send joints with a ndof > 0
        //         if let Some(_) = joint.downcast_ref::<FixedJoint<f32>>() {
        //             return Some(0.0);
        //         }
        //
        //         None
        //     })
        //     .collect();
        //
        // return state;
    }
}
