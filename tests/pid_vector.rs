#![cfg(feature = "vqm")]
#![allow(unused)]

use vqm::Vector3f32;

use pidsk_controller::{PidController, PidErrors, PidGains, PidLimits};

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
        assert_eq!(Vector3f32 { x: 2.0, y: 3.0, z: 5.0 }, pid_gains.kp);

        let error = pid.error();
        assert_eq!(error.p, Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
        assert_eq!(error.i, Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
        assert_eq!(error.d, Vector3f32 { x: 0.0, y: 0.0, z: 0.0 });
    }
}
