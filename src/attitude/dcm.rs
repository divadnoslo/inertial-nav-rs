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
        angle.cos(),  0.0, angle.sin(), 
        0.0,          1.0, 0.0, 
        -angle.sin(), 0.0, angle.cos())
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
        angle.cos(),  0.0, -angle.sin(), 
        0.0,          1.0, 0.0, 
        angle.sin(),  0.0, angle.cos())
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

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use std::f64::consts::FRAC_PI_2; // PI / 2 (90 degrees)

    #[test]
    fn test_rotate_x_90_degrees() {
        // Rotate the +Y unit vector by +90 degrees around the X-axis
        let v = Vector3::new(0.0, 1.0, 0.0);
        let rotated = rotate_x(FRAC_PI_2) * v;

        // Expect the vector to end up aligned with +Z: (0, 0, 1)
        let expected = Vector3::new(0.0, 0.0, 1.0);

        // Use a small epsilon tolerance for floating-point comparisons
        let eps = 1e-16;
        assert!((rotated.x - expected.x).abs() < eps);
        assert!((rotated.y - expected.y).abs() < eps);
        assert!((rotated.z - expected.z).abs() < eps);
    }

    #[test]
    fn test_rotate_z_30_degrees() {
        // Rotate the +X unit vector by +30 degrees around the Z-axis
        let angle: f64 = 30.0 * (180.0/std::f64::consts::PI);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let rotated = rotate_z(angle) * v;

        // Expected vector
        let x: f64 = angle.cos();
        let y: f64 = angle.sin();
        let z: f64 = 0.0;
        let expected = Vector3::new(x, y, z);

        // Use a small epsilon tolerance for floating-point comparisons
        let eps: f64 = 1e-16;
        assert!((rotated.x - expected.x).abs() < eps);
        assert!((rotated.y - expected.y).abs() < eps);
        assert!((rotated.z - expected.z).abs() < eps);
    }

    #[test]
    fn test_rotate_y_60_degrees() {
        // Rotate the +X unit vector by -60 degrees around the Y-axis
        let angle: f64 = -60.0 * (180.0/std::f64::consts::PI);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let rotated = rotate_y(angle) * v;

        // Expected vector
        let x: f64 = angle.cos();
        let y: f64 = 0.0;
        let z: f64 = -angle.sin();
        let expected = Vector3::new(x, y, z);

        // Use a small epsilon tolerance for floating-point comparisons
        let eps: f64 = 1e-16;
        assert!((rotated.x - expected.x).abs() < eps);
        assert!((rotated.y - expected.y).abs() < eps);
        assert!((rotated.z - expected.z).abs() < eps);
    }

    #[test]
    fn test_x_rotate_transform() {
        let angle: f64 = 45.0;
        let r = rotate_x(angle);
        let c = transform_x(angle);

        let delta = r - c.transpose();

        let eps: f64 = 1e-16;
        assert!(delta.m11.abs() < eps);
        assert!(delta.m12.abs() < eps);
        assert!(delta.m13.abs() < eps);
        assert!(delta.m21.abs() < eps);
        assert!(delta.m22.abs() < eps);
        assert!(delta.m23.abs() < eps);
        assert!(delta.m31.abs() < eps);
        assert!(delta.m32.abs() < eps);
        assert!(delta.m33.abs() < eps);
    }

    #[test]
    fn test_y_rotate_transform() {
        let angle: f64 = -45.0;
        let r = rotate_y(angle);
        let c = transform_y(angle);

        let delta = r - c.transpose();

        let eps: f64 = 1e-16;
        assert!(delta.m11.abs() < eps);
        assert!(delta.m12.abs() < eps);
        assert!(delta.m13.abs() < eps);
        assert!(delta.m21.abs() < eps);
        assert!(delta.m22.abs() < eps);
        assert!(delta.m23.abs() < eps);
        assert!(delta.m31.abs() < eps);
        assert!(delta.m32.abs() < eps);
        assert!(delta.m33.abs() < eps);
    }

    #[test]
    fn test_z_rotate_transform() {
        let angle: f64 = 75.0;
        let r = rotate_z(angle);
        let c = transform_z(angle);

        let delta = r - c.transpose();

        let eps: f64 = 1e-16;
        assert!(delta.m11.abs() < eps);
        assert!(delta.m12.abs() < eps);
        assert!(delta.m13.abs() < eps);
        assert!(delta.m21.abs() < eps);
        assert!(delta.m22.abs() < eps);
        assert!(delta.m23.abs() < eps);
        assert!(delta.m31.abs() < eps);
        assert!(delta.m32.abs() < eps);
        assert!(delta.m33.abs() < eps);
    }

}