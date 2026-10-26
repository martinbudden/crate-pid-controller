#![cfg(feature = "vqm")]
#![allow(unused)]

use pidsk_controller::{PidController, PidErrors, PidGains, PidLimits};
use signal_filters::{Pt1FilterVector3f32, UpdateFilter};
use vqm::{Quaternionf32, Vector3f32};

pub type PidErrorsVector3f32 = PidErrors<Vector3f32>;
pub type PidLimitsVector3f32 = PidLimits<Vector3f32>;
pub type PidGainsVector3f32 = PidGains<Vector3f32>;
pub type PidControllerVector3f32 = PidController<Vector3f32, f32>;

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    // Use assert_eq!(actual, expected) which is significantly more common across the wider software engineering landscape.

    use super::*;

    macro_rules! assert_near {
        ($left:expr, $right:expr) => {
            approx::assert_abs_diff_eq!($left, $right, epsilon = 4e-6);
        };
    }
    #[test]
    fn default() {
        let pid: PidControllerVector3f32 = PidControllerVector3f32::default();
        let pid_gains = pid.gains();
        assert_eq!(pid_gains.kp, Vector3f32 { x: 1.0, y: 1.0, z: 1.0 });
        assert_eq!(pid_gains.ki, Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
        assert_eq!(pid_gains.kd, Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
        assert_eq!(pid.setpoint(), Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
    }

    #[test]
    fn test_pid_init() {
        let pid = PidControllerVector3f32::new().with_kp(Vector3f32 { x: 2.0, y: 3.0, z: 5.0 });

        let pid_gains = pid.gains();
        assert_eq!(pid_gains.kp, Vector3f32 { x: 2.0, y: 3.0, z: 5.0 });
        assert_eq!(pid_gains.ki, Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
        assert_eq!(pid_gains.kd, Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
        let error = pid.error();
        assert_eq!(error.p, Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
        assert_eq!(error.i, Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
        assert_eq!(error.d, Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
    }

    #[test]
    fn test_pid_orientation() {
        let dt = 0.01;
        let mut pt1_filter = Pt1FilterVector3f32::new().with_k(0.9);
        let initial_orientation = Quaternionf32::identity();
        let target_orientation = Quaternionf32::identity(); // setpoint

        let mut pid = PidControllerVector3f32::new()
            .with_kp(Vector3f32::new(4.5, 4.5, 2.0))
            .with_ki(Vector3f32::new(0.1, 0.1, 0.05))
            .with_kd(Vector3f32::new(1.2, 1.2, 0.5));

        pid.set_setpoint(target_orientation.imaginary());

        let gyro_dps = Vector3f32 {
            x: 100.0,
            y: 110.0,
            z: 20.0,
        };

        let gyro = gyro_dps.to_radians() * dt * 0.5;
        let gyro_q = Quaternionf32::from(gyro);

        let orientation = (initial_orientation * gyro_q).normalize();
        let measurement = orientation.imaginary();
        let measurement_delta_filtered = (measurement - pid.previous_measurement()).filter_using(&mut pt1_filter);

        let error = (target_orientation * orientation.conjugate()).normalize().imaginary();

        let output = pid.update_delta_error(measurement, measurement_delta_filtered, error, dt);
    }
}
