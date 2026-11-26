#![cfg(feature = "vqm")]
#![allow(unused)]
#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    // Use assert_eq!(actual, expected) which is significantly more common across the wider software engineering landscape.
    #[cfg(feature = "nalgebra")]
    use nalgebra::{Quaternion, UnitQuaternion, Vector3};
    use pidsk_controller::PidController;
    #[cfg(feature = "vqm")]
    use vqm::{Quaternion, UnitQuaternion, Vector3};

    use super::*;

    macro_rules! assert_near {
        ($left:expr, $right:expr) => {
            approx::assert_abs_diff_eq!($left, $right, epsilon = 4e-6);
        };
    }

    #[test]
    fn default() {
        pub type PidControllerVector3f32 = PidController<Vector3<f32>, f32>;

        let pid: PidControllerVector3f32 = PidControllerVector3f32::default();
        let pid_gains = pid.gains();
        assert_eq!(pid_gains.kp, Vector3::<f32>::new(1.0, 1.0, 1.0));
        assert_eq!(pid_gains.ki, Vector3::<f32>::new(0.0, 0.0, 0.0));
        assert_eq!(pid_gains.kd, Vector3::<f32>::new(0.0, 0.0, 0.0));
        assert_eq!(pid.setpoint(), Vector3::<f32>::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn test_pid_init() {
        pub type PidControllerVector3f32 = PidController<Vector3<f32>, f32>;

        let pid = PidControllerVector3f32::new().with_kp(Vector3::<f32>::new(2.0, 3.0, 5.0));

        let pid_gains = pid.gains();
        assert_eq!(pid_gains.kp, Vector3::<f32>::new(2.0, 3.0, 5.0));
        assert_eq!(pid_gains.ki, Vector3::<f32>::new(0.0, 0.0, 0.0));
        assert_eq!(pid_gains.kd, Vector3::<f32>::new(0.0, 0.0, 0.0));
        let error = pid.error();
        assert_eq!(error.p, Vector3::<f32>::new(0.0, 0.0, 0.0));
        assert_eq!(error.i, Vector3::<f32>::new(0.0, 0.0, 0.0));
        assert_eq!(error.d, Vector3::<f32>::new(0.0, 0.0, 0.0));
    }

    #[test]
    fn angle_controller() {
        use signal_filters::{Pt1FilterVector3f32, UpdateFilter};

        pub type PidControllerVector3f32 = PidController<Vector3<f32>, f32>;

        let dt = 0.01;
        let mut pt1_filter = Pt1FilterVector3f32::new().with_k(0.9);
        let initial_orientation = Quaternion::<f32>::new(1.0, 0.0, 0.0, 0.0);
        let target_orientation = Quaternion::<f32>::new(1.0, 0.0, 0.0, 0.0); // setpoint

        let mut pid = PidControllerVector3f32::new()
            .with_kp(Vector3::<f32>::new(4.5, 4.5, 2.0))
            .with_ki(Vector3::<f32>::new(0.1, 0.1, 0.05))
            .with_kd(Vector3::<f32>::new(1.2, 1.2, 0.5));

        pid.set_setpoint(target_orientation.imag());

        let gyro_dps = Vector3::<f32>::new(100.0_f32.to_radians(), 110.0_f32.to_radians(), 20.0_f32.to_radians());

        let gyro = gyro_dps * dt * 0.5;
        let gyro_q = Quaternion::<f32>::from(gyro);

        let orientation = (initial_orientation * gyro_q).normalize();
        let measurement = orientation.imag();
        let measurement_delta_filtered = (measurement - pid.previous_measurement()).filter_using(&mut pt1_filter);

        let error = (target_orientation * orientation.conjugate()).normalize().imag();

        //let output = pid.update_delta_error(measurement, measurement_delta_filtered, error, dt);
    }

    #[test]
    fn rate_controller() {
        use signal_filters::{Pt1FilterVector3Vector3f32, Pt1FilterVector3f32, UpdateFilter};
        pub type PidControllerVector3f32 = PidController<Vector3<f32>, f32>;

        let dt = 0.01;
        let mut gyro_filter = Pt1FilterVector3f32::new().with_k(0.9);
        let mut dterm_filter = Pt1FilterVector3Vector3f32::new().with_k(Vector3::<f32>::new(0.9, 0.9, 1.0));

        // Create our vector PID controller and set the gains.
        // Note the yaw kd value is zero, since we use a PD controller for yaw.
        let mut pid_controller = PidControllerVector3f32::new()
            .with_kp(Vector3::<f32>::new(4.5, 4.5, 2.0))
            .with_ki(Vector3::<f32>::new(0.1, 0.1, 0.05))
            .with_kd(Vector3::<f32>::new(1.2, 1.2, 0.0));

        // Create a mock gyro measurement (in practice this comes from the IMU).
        let gyro_measurement_dps = Vector3::<f32>::new(100.0, 110.0, 20.0);
        // Filter it and convert radians per second.
        let gyro_measurement_filtered_rps = gyro_measurement_dps.filter_using(&mut gyro_filter).to_radians();
        // Convert to quaternion.
        let gyro_measurement_q = Quaternion::<f32>::from(gyro_measurement_filtered_rps);

        // Create a mock target rate (in practice this comes from the joysticks).
        let target_rates_q = UnitQuaternion::<f32>::from_roll_pitch_yaw_degrees(200.0, 100.0, 50.0);
        // Use it to set the PID controller setpoint.
        pid_controller.set_setpoint(target_rates_q.imag());

        // Calculate the measurement delta and filter it.
        let measurement = gyro_measurement_q.imag();
        let measurement_delta_filtered =
            (measurement - pid_controller.previous_measurement()).filter_using(&mut dterm_filter);

        // Calculate the error: setpoint - measurement, but in Quaternion space
        let error = (Quaternion::<f32>::from(pid_controller.setpoint()) * gyro_measurement_q.conjugate())
            .normalize()
            .imag();

        #[cfg(feature = "vqm")]
        let output = pid_controller.update_delta_error(measurement, measurement_delta_filtered, error, dt);
    }
}
