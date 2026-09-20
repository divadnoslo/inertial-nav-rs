// src/attitude/dcm.rs
use nalgebra::Matrix3;

/// Produces an active rotation matrix about the x-axis.
///
/// Rotates a 3D vector counterclockwise around the x-axis according
/// to the right-hand rule.
///
/// # Arguments
/// * `angle` - The angle of rotation [radians]
/// 
/// # Returns
/// * `Matrix3<f64>` - A $3 \times 3$ active rotation matrix belonging to $\text{SO}(3)$.
pub fn rotate_x(angle: f64) -> Matrix3<f64> {
    Matrix3::new(
        1.0, 0.0,         0.0, 
        0.0, angle.cos(), -angle.sin(), 
        0.0, angle.sin(), angle.cos())
}

/// Produces an active rotation matrix about the y-axis.
///
/// Rotates a 3D vector counterclockwise around the y-axis according
/// to the right-hand rule.
///
/// # Arguments
/// * `angle` - The angle of rotation [radians]
/// 
/// # Returns
/// * `Matrix3<f64>` - A $3 \times 3$ active rotation matrix belonging to $\text{SO}(3)$.
pub fn rotate_y(angle: f64) -> Matrix3<f64> {
    Matrix3::new(
        angle.cos(), 0.0, -angle.sin(), 
        0.0,         1.0, 0.0, 
        angle.sin(), 0.0, angle.cos())
}

/// Produces an active rotation matrix about the z-axis.
///
/// Rotates a 3D vector counterclockwise around the z-axis according
/// to the right-hand rule.
///
/// # Arguments
/// * `angle` - The angle of rotation [radians]
/// 
/// # Returns
/// * `Matrix3<f64>` - A $3 \times 3$ active rotation matrix belonging to $\text{SO}(3)$.
pub fn rotate_z(angle: f64) -> Matrix3<f64> {
    Matrix3::new(
        angle.cos(), -angle.sin(), 0.0, 
        angle.sin(), angle.cos(),  0.0, 
        0.0,         0.0,          1.0)
}

/// Produces a passive rotation matrix about the x-axis.
///
/// Rotates a 3D vector clockwise around the x-axis according
/// to the right-hand rule.
///
/// # Arguments
/// * `angle` - The angle of rotation [radians]
/// 
/// # Returns
/// * `Matrix3<f64>` - A $3 \times 3$ active rotation matrix belonging to $\text{SO}(3)$.
pub fn transform_x(angle: f64) -> Matrix3<f64> {
    Matrix3::new(
        1.0, 0.0,          0.0, 
        0.0, angle.cos(),  angle.sin(), 
        0.0, -angle.sin(), angle.cos())
}

/// Produces a passive rotation matrix about the y-axis.
///
/// Rotates a 3D vector clockwise around the y-axis according
/// to the right-hand rule.
///
/// # Arguments
/// * `angle` - The angle of rotation [radians]
/// 
/// # Returns
/// * `Matrix3<f64>` - A $3 \times 3$ active rotation matrix belonging to $\text{SO}(3)$.
pub fn transform_y(angle: f64) -> Matrix3<f64> {
    Matrix3::new(
        angle.cos(),  0.0, angle.sin(), 
        0.0,          1.0, 0.0, 
        -angle.sin(), 0.0, angle.cos())
}

/// Produces a passive rotation matrix about the z-axis.
///
/// Rotates a 3D vector clockwise around the z-axis according
/// to the right-hand rule.
///
/// # Arguments
/// * `angle` - The angle of rotation [radians]
/// 
/// # Returns
/// * `Matrix3<f64>` - A $3 \times 3$ active rotation matrix belonging to $\text{SO}(3)$.
pub fn transform_z(angle: f64) -> Matrix3<f64> {
    Matrix3::new(
        angle.cos(),  angle.sin(), 0.0, 
        -angle.sin(), angle.cos(),  0.0, 
        0.0,          0.0,          1.0)
}