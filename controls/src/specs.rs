use nphysics3d::{
    joint::{FixedJoint, PrismaticJoint, RevoluteJoint, UnitJoint},
    math::{AngularVector, Isometry, Translation, Vector},
    nalgebra::UnitQuaternion,
};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct Link {
    pub vertices: Vec<f32>,
    pub uv: Vec<f32>,
    pub indices: Vec<usize>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct BasePosition {
    pub index: usize,
    pub position: [f32; 3],
    pub direction: [f32; 3],
}

#[derive(Serialize, Deserialize, Debug)]
pub struct AttachmentPosition {
    pub index: usize,
    pub position: [f32; 3],
    pub quaternion: [f32; 4],
}

#[derive(Debug, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JointType {
    Rotational,
    Sliding,
    Static,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Joint {
    pub r#type: JointType,
    pub name: String,
    pub base: BasePosition,
    pub attachment: AttachmentPosition,
}

impl Joint {
    pub fn to_phys_joint(&self) -> Box<dyn nphysics3d::joint::Joint<f32>> {
        let pos = {
            let [x, y, z] = self.base.position;
            let trans = Translation::new(x, y, z);
            let [x, y, z] = self.base.direction;
            let quat = UnitQuaternion::new(Vector::<f32>::new(x, y, z));
            Isometry::from_parts(trans, quat)
        };

        match self.r#type {
            JointType::Rotational => Box::new(RevoluteJoint::new(AngularVector::z_axis(), 0.0)),
            JointType::Sliding => Box::new(PrismaticJoint::new(AngularVector::z_axis(), 0.0)),
            JointType::Static => Box::new(FixedJoint::new(pos)),
        }
    }
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Specs {
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
}
