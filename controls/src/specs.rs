use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug)]
pub struct Link {
    pub vertices: Vec<f32>,
    pub indices: Vec<u32>,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Joint1 {
    pub index: usize,
    pub position: [f32; 3],
    pub direction: [f32; 3],
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Joint2 {
    pub index: usize,
    pub position: [f32; 3],
    pub quaternion: [f32; 4],
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Joint {
    pub r#type: String,
    pub link1: Joint1,
    pub link2: Joint2,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Specs {
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
}
