use anyhow::{Context, Result};

use k::{Chain, Constraints, InverseKinematicsSolver, SerialChain, Translation3};

use nphysics3d::object::{DefaultBodySet, DefaultColliderSet};

pub struct Robot {
    // multibody: DefaultBodyHandle,
    specs: urdf_rs::Robot,
    chain: k::Chain<f32>,
    serial_chain: k::SerialChain<f32>,
}

impl Robot {
    pub fn default() -> Robot {
        // TODO: get robot specs from disk
        let chain = Chain::from_root(k::NodeBuilder::new().into_node());
        let jp: Vec<f32> = (0..chain.dof()).map(|_| 0.0).collect();
        chain.set_joint_positions(&jp).unwrap();
        chain.update_transforms();
        Robot {
            // FIXME: always assume chain is in series
            serial_chain: SerialChain::try_new(chain.clone()).unwrap(),
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
        _bodies: &mut DefaultBodySet<f32>,
        _colliders: &mut DefaultColliderSet<f32>,
        specs: urdf_rs::Robot,
    ) -> Result<Robot> {
        let chain = k::Chain::from(&specs);
        let jp: Vec<f32> = (0..chain.dof()).map(|_| 0.0).collect();
        chain.set_joint_positions(&jp)?;
        chain.update_transforms();
        println!("{chain}");

        let joint = specs
            .joints
            .get(0)
            .context("need more than 1 joint")?
            .to_phys_joint();
        let mut multibody_desc: MultibodyDesc<f32> = MultibodyDesc::from_boxed_joint(joint);

        // FIXME: assumes all joints are in order and is in a chain
        let mut last = &mut multibody_desc;
        // for joint in specs.joints.iter().skip(1) {
        //     let phys_joint = joint.to_phys_joint();
        //     last = last
        //         .add_child_desc(MultibodyDesc::from_boxed_joint(phys_joint))
        //         .set_name(joint.name.clone());
        //     break;
        // }

        let multibody = multibody_desc.build();
        let multibody = bodies.insert(multibody);

        // for i in 1..2 {
        //     let link = &specs.links[i];
        //
        //     if link.vertices.len() == 0 {
        //         continue;
        //     }
        //     let vertices: Vec<Point3<f32>> = link
        //         .vertices
        //         .chunks(3)
        //         .map(|c| match c {
        //             &[x, y, z, ..] => Point3::new(x, y, z),
        //             _ => panic!(),
        //         })
        //         .collect();
        //     let indices = link
        //         .indices
        //         .chunks(3)
        //         .map(|c| match c {
        //             &[x, y, z, ..] => Point3::new(x, y, z),
        //             _ => panic!(),
        //         })
        //         .collect();
        //     let uv = link
        //         .uv
        //         .chunks(2)
        //         .map(|c| match c {
        //             &[x, y, ..] => Point2::new(x, y),
        //             _ => panic!(),
        //         })
        //         .collect();
        //     let shape = ShapeHandle::new(TriMesh::new(vertices, indices, Some(uv)));
        //     let collider = ColliderDesc::new(shape)
        //         // lets say each link is made of steel
        //         .density(7.85)
        //         .build(BodyPartHandle(multibody, i - 1));
        //     colliders.insert(collider);
        // }

        // specs
        //     .links
        //     .into_iter()
        //     .map(|link| {
        //         let rg = RigidBodyBuilder::kinematic_position_based()
        //             .lock_translations()
        //             .lock_rotations()
        //             .build();
        //         let rg_handle = rigid_body_set.insert(rg);
        //         let vertices: Vec<Point3<f32>> = link
        //             .vertices
        //             .chunks(3)
        //             .map(|c| match c {
        //                 &[x, y, z, ..] => Point3::new(x, y, z),
        //                 _ => panic!(),
        //             })
        //             .collect();
        //         let indices = link
        //             .indices
        //             .chunks(3)
        //             .map(|c| match c {
        //                 &[x, y, z, ..] => [x, y, z],
        //                 _ => panic!(),
        //             })
        //             .collect();
        //         if vertices.len() > 0 {
        //             let link_collider = ColliderBuilder::trimesh(vertices, indices)
        //                 // lets say each link is made of steel
        //                 .density(7.85)
        //                 .build();
        //             let col_handle =
        //                 colliders.insert_with_parent(link_collider, rg_handle, &mut rigid_body_set);
        //         }
        //
        //         rg_handle
        //     })
        //     .collect();
        //
        // let joint_handles = specs
        //     .joints
        //     .into_iter()
        //     .map(|joint_spec| {
        //         let rg1 = rg_handles
        //             .get(joint_spec.link1.index)
        //             .context("invalid index")?
        //             .clone();
        //         let rg2 = rg_handles
        //             .get(joint_spec.link1.index)
        //             .context("invalid index")?
        //             .clone();
        //
        //         let frame1 = {
        //             let [x, y, z] = joint_spec.link1.position;
        //             let trans = Translation3::new(x, y, z);
        //             let [x, y, z] = joint_spec.link1.direction;
        //             let quat = UnitQuaternion::new(Vector3::<f32>::new(x, y, z));
        //             Isometry3::from_parts(trans, quat)
        //         };
        //
        //         let frame2 = {
        //             let [x, y, z] = joint_spec.link2.position;
        //             let trans = Translation3::new(x, y, z);
        //             let [x, y, z, w] = joint_spec.link2.quaternion;
        //             let quat = UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
        //             Isometry3::from_parts(trans, quat)
        //         };
        //
        //         let handle = match joint_spec.r#type.as_str() {
        //             "static" => {
        //                 let joint = FixedJointBuilder::new()
        //                     .local_frame1(frame1)
        //                     .local_frame2(frame2);
        //                 joint_set.insert(rg1, rg2, joint, true)
        //             }
        //             "rotational" => {
        //                 println!("{:?}", frame1.rotation.axis());
        //                 let axis = frame1.rotation.axis().context("invalid axis")?;
        //                 let joint = RevoluteJointBuilder::new(axis)
        //                     .local_anchor1(frame1.translation.vector.into())
        //                     .local_anchor2(frame2.translation.vector.into());
        //                 joint_set.insert(rg1, rg2, joint, true)
        //             }
        //             "sliding" => {
        //                 let axis = frame1.rotation.axis().context("invalid axis")?;
        //                 let mut joint = PrismaticJoint::new(axis);
        //                 joint.set_local_anchor1(frame1.translation.vector.into());
        //                 joint.set_local_anchor2(frame2.translation.vector.into());
        //                 joint_set.insert(rg1, rg2, joint, true)
        //             }
        //             _ => return Err(anyhow!("unknown joint type")),
        //         }
        //         .context("invalid multibody joint")?;
        //
        //         Ok(handle)
        //     })
        //     .collect::<Result<Vec<MultibodyJointHandle>>>()?;

        Ok(Robot {
            // FIXME: always assume chain is in series
            serial_chain: SerialChain::try_new(chain.clone())
                .context("urdf robot is not in series")?,
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

    pub fn state(&self) -> Vec<f32> {
        todo!()
    }
}
