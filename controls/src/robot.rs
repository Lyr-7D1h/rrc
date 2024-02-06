use std::borrow::Borrow;

use anyhow::{anyhow, Context, Result};

use k::{Chain, Constraints, InverseKinematicsSolver, SerialChain, Translation3};

use nphysics3d::{
    joint::{FixedJoint, Joint, PrismaticJoint, RevoluteJoint},
    math::{AngularVector, Isometry, Translation, Vector},
    nalgebra::{UnitQuaternion, Vector3},
    ncollide3d::shape::{Cuboid, Shape, ShapeHandle},
    object::{
        BodyPartHandle, ColliderDesc, DefaultBodyHandle, DefaultBodySet, DefaultColliderSet,
        MultibodyDesc,
    },
};

pub type State = Vec<f32>;

pub struct Robot {
    multibody: DefaultBodyHandle,
    specs: urdf_rs::Robot,
    chain: k::Chain<f32>,
    serial_chain: k::SerialChain<f32>,
}

fn to_phys_joint(joint: &urdf_rs::Joint) -> Result<Box<dyn Joint<f32>>> {
    let pos = {
        let origin = &joint.origin;
        let [x, y, z] = origin.xyz.map(|x| x as f32);
        let trans = Translation::new(x, y, z);
        let [r, p, y] = origin.rpy.map(|x| x as f32);
        let quat = UnitQuaternion::from_euler_angles(r, p, y);
        Isometry::from_parts(trans, quat)
    };

    let phys_joint: Box<dyn Joint<f32>> = match joint.joint_type {
        urdf_rs::JointType::Revolute => Box::new(RevoluteJoint::new(AngularVector::z_axis(), 0.0)),
        urdf_rs::JointType::Prismatic => {
            Box::new(PrismaticJoint::new(AngularVector::z_axis(), 0.0))
        }
        urdf_rs::JointType::Fixed => Box::new(FixedJoint::new(pos)),
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

impl Robot {
    // TODO: get robot specs from disk
    pub fn default(
        _bodies: &mut DefaultBodySet<f32>,
        _colliders: &mut DefaultColliderSet<f32>,
    ) -> Robot {
        let chain = Chain::from_root(k::NodeBuilder::new().into_node());
        let jp: Vec<f32> = (0..chain.dof()).map(|_| 0.0).collect();
        chain.set_joint_positions(&jp).unwrap();
        chain.update_transforms();

        Robot {
            // FIXME: always assume chain is in series
            serial_chain: SerialChain::try_new(chain.clone()).unwrap(),
            multibody: DefaultBodyHandle::from_raw_parts(0, 0),
            chain,
            specs: urdf_rs::Robot {
                name: "".into(),
                links: vec![],
                joints: vec![],
                materials: vec![],
            },
        }
    }

    pub fn from_specs(
        bodies: &mut DefaultBodySet<f32>,
        colliders: &mut DefaultColliderSet<f32>,
        specs: urdf_rs::Robot,
    ) -> Result<Robot> {
        let chain = k::Chain::from(&specs);
        let jp: Vec<f32> = (0..chain.dof()).map(|_| 0.0).collect();
        chain.set_joint_positions(&jp)?;
        chain.update_transforms();
        println!("{chain}");

        // FIXME: assumes first one is the base
        let joint = to_phys_joint(specs.joints.get(0).context("need more than 1 joint")?)?;
        let mut multibody_desc: MultibodyDesc<f32> = MultibodyDesc::from_boxed_joint(joint);

        // FIXME: assumes all joints are in order and is in a chain
        let mut last = &mut multibody_desc;
        for joint in specs.joints.iter().skip(1) {
            let phys_joint = to_phys_joint(joint)?;
            last = last
                .add_child_desc(MultibodyDesc::from_boxed_joint(phys_joint))
                .set_name(joint.name.clone());
        }

        let multibody = multibody_desc.build();
        let multibody = bodies.insert(multibody);

        // FIXME: assumes first one does not have collision
        for i in 1..specs.links.len() {
            let link = &specs.links[i];

            if let Some(collision) = link.collision.get(0) {
                let handle = to_phys_shape_handle(&collision.geometry)?;
                let collider = ColliderDesc::new(handle)
                    // lets say each link is made of steel
                    .density(7.85)
                    .build(BodyPartHandle(multibody, i - 1));
                colliders.insert(collider);
            }
        }

        Ok(Robot {
            // TODO: allow non serial chains
            serial_chain: SerialChain::try_new(chain.clone())
                .context("urdf robot is not in series")?,
            multibody,
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
            position_x: true,
            position_y: true,
            position_z: true,
            rotation_x: false,
            rotation_y: false,
            rotation_z: false,
            ignored_joint_names: vec![],
        };
        solver.solve_with_constraints(&serial, &target, &constraints)?;
        self.chain.update_transforms();
        let solved_pose = end_effector.world_transform().unwrap();
        for (i, trans) in serial.update_transforms().iter().enumerate() {}
        println!("sol: {solved_pose}");

        return Ok(());
    }

    pub fn state(&self) -> State {
        todo!()
    }
}
