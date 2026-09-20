// src/lib.rs

pub mod attitude;
//pub mod filtering;
//pub mod gravity;
//pub mod imu;

// Re-export core linear algebra types so users don't need nalgebra as a direct dependency
pub use nalgebra::{Vector3, Matrix3};