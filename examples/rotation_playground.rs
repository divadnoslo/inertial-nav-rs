use inertial_nav_rs::attitude::dcm::{rotate_x, rotate_y, rotate_z, transform_z};
use nalgebra::Vector3;
use std::f64::consts::FRAC_PI_2; // 90 degrees in radians

fn main() {
    println!("=== Inertial Nav RS: Rotation Playground ===\n");

    // 1. Define a unit vector along +X
    let v = Vector3::new(1.0, 0.0, 0.0);
    println!("Original vector v: {:.1}", v);

    // 2. Active rotation: Rotate v by +90 deg around Z-axis
    let rz = rotate_z(FRAC_PI_2);
    let v_rotated = rz * v;
    println!("\nActive rotate_z(+90°):");
    println!("Matrix:\n{:.1}", rz);
    println!("Rotated vector (expected +Y): {:.1}", v_rotated);

    // 3. Passive transformation vs Active rotation
    let tz = transform_z(FRAC_PI_2);
    let v_transformed = tz * v;
    println!("\nPassive transform_z(+90°):");
    println!("Matrix:\n{:.1}", tz);
    println!("Transformed vector (expected -Y): {:.1}", v_transformed);

    // 4. Verify Active vs Passive relationship (Transpose check)
    println!("\nIs transform_z equal to rotate_z.transpose()? {}", tz == rz.transpose());
}