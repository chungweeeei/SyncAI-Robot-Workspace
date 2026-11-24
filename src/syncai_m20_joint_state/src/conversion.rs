//! The vendor's raw joint readings -> M20.urdf's joint angles. Pure: no ROS types, so the unit
//! tests below run under `cargo test` with no robot.
//!
//! Everything here is DEEP Robotics' own conversion, copied from their RL deployment code
//! (`src/M20_sdk_deploy/interface/robot/hardware/` in sdk_deploy, at the SHA
//! third-party-sparse.repos pins): `M20Interface`'s `init_pos_offset` / `joint_dir` tables,
//! `DdsInterface::Handler`'s `raw * dir + offset`, and `M20Interface::ResetPositionOffset`'s
//! one-off ±360° correction. Change it only together with theirs.
//!
//! Why those angles are M20.urdf's: the vendor's MJCF (`M20_description/m20_mjcf/mjcf/M20.xml`)
//! has the same link offsets and the same 16 joint axes as `syncai_bringup/description/M20.urdf`,
//! and the policy runs on that MJCF. Checked on robot01 (2026-11-24) with forward kinematics on
//! M20.urdf: lying, the knees fold under the body; standing, every wheel is 0.48-0.50 m below its
//! hip and under it. M20.urdf's hipy / knee `<limit>`s are the MJCF's negated (an exporter
//! artefact); robot_state_publisher does not read limits, so the TF is unaffected.

/// Joints per message, in `/JOINTS_DATA`'s order: legs fl, fr, hl, hr, each hipx, hipy, knee,
/// wheel. The vendor's `robot_order`; the element's own `name` (ASCII "0".."15") is the same
/// index and is not read.
pub const JOINT_COUNT: usize = 16;

/// M20.urdf's joint names in `/JOINTS_DATA` order.
pub const JOINT_NAMES: [&str; JOINT_COUNT] = [
    "fl_hipx_joint",
    "fl_hipy_joint",
    "fl_knee_joint",
    "fl_wheel_joint",
    "fr_hipx_joint",
    "fr_hipy_joint",
    "fr_knee_joint",
    "fr_wheel_joint",
    "hl_hipx_joint",
    "hl_hipy_joint",
    "hl_knee_joint",
    "hl_wheel_joint",
    "hr_hipx_joint",
    "hr_hipy_joint",
    "hr_knee_joint",
    "hr_wheel_joint",
];

/// The vendor's `joint_dir`: the motor's sense against the joint's.
const DIR: [f64; JOINT_COUNT] = [
    1.0, 1.0, -1.0, 1.0, //
    1.0, -1.0, 1.0, -1.0, //
    -1.0, 1.0, -1.0, 1.0, //
    -1.0, -1.0, 1.0, -1.0,
];

/// The vendor's `init_pos_offset`, degrees: where the motor's zero sits in the joint's angle.
const OFFSET_DEG: [f64; JOINT_COUNT] = [
    -25.0, -131.0, 160.0, 0.0, //
    25.0, -131.0, 160.0, 0.0, //
    -25.0, 131.0, -160.0, 0.0, //
    25.0, 131.0, -160.0, 0.0,
];

/// `ResetPositionOffset`'s windows, degrees: a hipy angle beyond ±140° or a knee one beyond ±164°
/// on the first reading is taken to be a turn off, and the offset moves by 360° for good.
const HIPY_WRAP_DEG: f64 = 140.0;
const KNEE_WRAP_DEG: f64 = 164.0;

fn is_hipy(i: usize) -> bool {
    i % 4 == 1
}

fn is_knee(i: usize) -> bool {
    i % 4 == 2
}

/// The offsets in effect, fixed from the first reading as the vendor's controller fixes them at
/// start-up. Not re-judged per reading: a hipy can legitimately pass 140° (its range reaches
/// 148°), where a per-reading wrap would flip it a full turn.
#[derive(Debug, Clone, PartialEq)]
pub struct Calibration {
    offset: [f64; JOINT_COUNT],
}

impl Calibration {
    pub fn from_first_reading(raw_position: &[f64; JOINT_COUNT]) -> Self {
        let mut offset = OFFSET_DEG.map(f64::to_radians);
        for i in 0..JOINT_COUNT {
            let window = if is_hipy(i) {
                HIPY_WRAP_DEG
            } else if is_knee(i) {
                KNEE_WRAP_DEG
            } else {
                continue;
            }
            .to_radians();
            let q = raw_position[i] * DIR[i] + offset[i];
            if q < -window {
                offset[i] += std::f64::consts::TAU;
            } else if q > window {
                offset[i] -= std::f64::consts::TAU;
            }
        }
        Self { offset }
    }

    /// Joint angles, rad. The wheels' are unbounded (continuous joints; the vendor applies no
    /// offset to them).
    pub fn positions(&self, raw_position: &[f64; JOINT_COUNT]) -> [f64; JOINT_COUNT] {
        std::array::from_fn(|i| raw_position[i] * DIR[i] + self.offset[i])
    }

    /// Whether the start-up correction moved any offset, for the log.
    pub fn wrapped_joints(&self) -> Vec<&'static str> {
        (0..JOINT_COUNT)
            .filter(|&i| (self.offset[i] - OFFSET_DEG[i].to_radians()).abs() > 1e-9)
            .map(|i| JOINT_NAMES[i])
            .collect()
    }
}

/// Joint velocities (rad/s) or efforts (N·m): only the sense changes, as in the vendor's handler.
pub fn directed(raw: &[f64; JOINT_COUNT]) -> [f64; JOINT_COUNT] {
    std::array::from_fn(|i| raw[i] * DIR[i])
}

#[cfg(test)]
mod tests {
    use super::*;

    // /JOINTS_DATA positions recorded on robot01, 2026-11-24
    const LYING: [f64; JOINT_COUNT] = [
        -0.000_307_156_762_573_868_04,
        1.206_630_825_996_399,
        0.031_211_888_417_601_585,
        -16.802_141_189_575_195,
        -0.005_637_110_210_955_143,
        -1.218_736_290_931_701_7,
        -0.033_660_069_108_009_34,
        29.324_600_219_726_562,
        -0.432_982_563_972_473_14,
        -1.113_663_673_400_879,
        -0.031_139_619_648_456_573,
        8.510_238_647_460_938,
        0.438_213_109_970_092_8,
        1.108_451_247_215_271,
        0.031_229_952_350_258_827,
        17.575_872_421_264_65,
    ];
    const STANDING: [f64; JOINT_COUNT] = [
        0.531_993_329_524_993_9,
        2.052_143_335_342_407_2,
        2.330_313_205_718_994,
        -16.114_469_528_198_242,
        -0.532_300_472_259_521_5,
        -2.019_260_406_494_140_6,
        -2.250_517_368_316_650_4,
        28.410_093_307_495_117,
        -0.524_983_048_439_025_9,
        -2.011_924_743_652_343_8,
        -2.230_010_509_490_967,
        7.876_287_937_164_307,
        0.480_328_857_898_712_16,
        1.989_710_569_381_713_9,
        2.189_520_835_876_465,
        17.986_787_796_020_508,
    ];

    fn assert_close(actual: f64, expected: f64, what: &str) {
        assert!(
            (actual - expected).abs() < 1e-3,
            "{what}: {actual} != {expected}"
        );
    }

    #[test]
    fn lying_folds_every_leg() {
        let cal = Calibration::from_first_reading(&LYING);
        assert!(cal.wrapped_joints().is_empty());
        let q = cal.positions(&LYING);
        // Front legs: hipy back, knee fully folded forward; hind legs mirrored
        assert_close(q[1], -1.080, "fl_hipy");
        assert_close(q[2], 2.761, "fl_knee");
        assert_close(q[5], -1.068, "fr_hipy");
        assert_close(q[6], 2.759, "fr_knee");
        assert_close(q[9], 1.173, "hl_hipy");
        assert_close(q[10], -2.761, "hl_knee");
        assert_close(q[13], 1.178, "hr_hipy");
        assert_close(q[14], -2.761, "hr_knee");
    }

    #[test]
    fn standing_is_near_the_zero_pose() {
        let cal = Calibration::from_first_reading(&STANDING);
        assert!(cal.wrapped_joints().is_empty());
        let q = cal.positions(&STANDING);
        assert_close(q[0], 0.096, "fl_hipx");
        assert_close(q[1], -0.234, "fl_hipy");
        assert_close(q[2], 0.462, "fl_knee");
        assert_close(q[4], -0.096, "fr_hipx");
        assert_close(q[9], 0.274, "hl_hipy");
        assert_close(q[10], -0.563, "hl_knee");
        assert_close(q[12], -0.044, "hr_hipx");
        assert_close(q[14], -0.603, "hr_knee");
    }

    #[test]
    fn wheels_have_no_offset() {
        let cal = Calibration::from_first_reading(&STANDING);
        let q = cal.positions(&STANDING);
        for i in [3, 7, 11, 15] {
            assert_close(q[i], STANDING[i] * DIR[i], JOINT_NAMES[i]);
        }
    }

    #[test]
    fn a_hipy_past_its_window_moves_a_full_turn_once() {
        // fl_hipy read at -150°: past -140°, so its offset gains 360° (the vendor's rule)
        let mut raw = STANDING;
        raw[1] = (-150.0_f64 + 131.0).to_radians();
        let cal = Calibration::from_first_reading(&raw);
        assert_eq!(cal.wrapped_joints(), vec!["fl_hipy_joint"]);
        assert_close(cal.positions(&raw)[1], 210.0_f64.to_radians(), "fl_hipy");

        // Fixed for good: a later reading inside the window keeps the moved offset
        let later = STANDING;
        assert_close(
            cal.positions(&later)[1],
            Calibration::from_first_reading(&later).positions(&later)[1] + std::f64::consts::TAU,
            "fl_hipy later",
        );
    }

    #[test]
    fn a_knee_past_its_window_moves_back_a_full_turn() {
        // hl_knee read at +170°: past +164°, so its offset loses 360°
        let mut raw = STANDING;
        // q = raw * dir + offset with dir -1 and offset -160°, so raw = -(170° + 160°)
        raw[10] = -(170.0_f64 + 160.0).to_radians();
        let cal = Calibration::from_first_reading(&raw);
        assert_eq!(cal.wrapped_joints(), vec!["hl_knee_joint"]);
        assert_close(
            cal.positions(&raw)[10],
            (-190.0_f64).to_radians(),
            "hl_knee",
        );
    }

    #[test]
    fn velocity_and_effort_only_change_sense() {
        let raw: [f64; JOINT_COUNT] = std::array::from_fn(|i| i as f64 + 1.0);
        let d = directed(&raw);
        for i in 0..JOINT_COUNT {
            assert_eq!(d[i], raw[i] * DIR[i]);
        }
    }

    #[test]
    fn names_follow_the_vendor_order() {
        for (i, name) in JOINT_NAMES.iter().enumerate() {
            let leg = ["fl", "fr", "hl", "hr"][i / 4];
            let part = ["hipx", "hipy", "knee", "wheel"][i % 4];
            assert_eq!(*name, format!("{leg}_{part}_joint"));
        }
    }
}
