//#![cfg(feature = "vqm")]
#![allow(unused)]

use pidsk_controller::{MinMaxClamp, PidController, PidErrors, PidGains, PidLimits};
use signal_filters::{Pt1Filter, Pt1FilterVector3f32, SignalFilter, UpdateFilter};
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
    fn angle_controller() {
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

        //let output = pid.update_delta_error(measurement, measurement_delta_filtered, error, dt);
    }

    #[test]
    fn rate_controller() {
        pub type Pt1FilterVector3Vector3f32 = Pt1Filter<Vector3f32, Vector3f32>;

        let dt = 0.01;
        let mut gyro_filter = Pt1FilterVector3f32::new().with_k(0.9);
        let mut dterm_filter = Pt1FilterVector3Vector3f32::new().with_k(Vector3f32::new(0.9, 0.9, 1.0));

        // Create our PID controller and set the gains.
        // Note the yaw kd value is zero, since we use a PD controller for yaw.
        let mut pid_controller = PidControllerVector3f32::new()
            .with_kp(Vector3f32::new(4.5, 4.5, 2.0))
            .with_ki(Vector3f32::new(0.1, 0.1, 0.05))
            .with_kd(Vector3f32::new(1.2, 1.2, 0.0));

        // Create a mock gyro measurement (in practice this comes from the IMU).
        let gyro_measurement_dps = Vector3f32 {
            x: 100.0,
            y: 110.0,
            z: 20.0,
        };
        // Filter it and convert radians per second.
        let gyro_measurement_filtered_rps = gyro_measurement_dps.filter_using(&mut gyro_filter).to_radians();
        // Convert to quaternion.
        let gyro_measurement_q = Quaternionf32::from(gyro_measurement_filtered_rps);

        // Create a mock target rate (in practice this comes from the joysticks).
        let target_rates_q = Quaternionf32::from_roll_pitch_yaw_degrees(200.0, 100.0, 50.0);
        // Use it to set the PID controller setpoint.
        pid_controller.set_setpoint(target_rates_q.imaginary());

        let measurement = gyro_measurement_q.imaginary();
        let measurement_delta_filtered =
            (measurement - pid_controller.previous_measurement()).filter_using(&mut dterm_filter);

        let error = (target_rates_q * gyro_measurement_q.conjugate())
            .normalize()
            .imaginary();

        #[cfg(feature = "vqm")]
        let output = pid_controller.update_delta_error(measurement, measurement_delta_filtered, error, dt);
    }
}
