//! m20_joint_state: the Lynx M20's `/JOINTS_DATA_10HZ` (drdds/msg/JointsData, published by the
//! vendor's stack on DDS domain 0) as `joint_states` (sensor_msgs/JointState) in M20.urdf's
//! joint names and convention, which is what robot_state_publisher needs to put the 16 leg and
//! wheel links on /tf. The conversion is the vendor's own (conversion.rs).
//!
//! The vendor publishes the same readings twice: `/JOINTS_DATA` at ~190 Hz, for its own
//! controller, and `/JOINTS_DATA_10HZ`, the same message 10 times a second. The leg links' TF
//! is for looking at, so this node reads the slow one (`joints_topic`).
//!
//! Started by syncai_bringup's bringup.launch.py for `robot_model: m20`, in the `<robot_id>`
//! namespace, so it publishes `/<robot_id>/joint_states` beside that robot_state_publisher.

mod conversion;

use std::sync::Arc;

use rclrs::*;
use ros_env::builtin_interfaces::msg::Time as TimeMsg;
use ros_env::drdds::msg::JointsData;
use ros_env::sensor_msgs::msg::JointState;
use ros_env::std_msgs::msg::Header;

use conversion::{Calibration, JOINT_COUNT, JOINT_NAMES, directed};

/// What the subscription callback owns.
struct Bridge {
    publisher: Publisher<JointState>,
    clock: Clock,
    logger: Logger,
    /// Fixed by the first reading (conversion.rs has why)
    calibration: Option<Calibration>,
    /// Minimum time between two publications, ns; 0 = every reading
    min_period_ns: i64,
    last_publish_ns: Option<i64>,
}

impl Bridge {
    fn on_reading(&mut self, msg: JointsData) {
        let now_ns = self.clock.now().nsec;
        if let Some(last) = self.last_publish_ns {
            if now_ns - last < self.min_period_ns {
                return;
            }
        }

        let joints = &msg.data.joints_data;
        let field = |f: fn(&ros_env::drdds::msg::JointData) -> f32| -> [f64; JOINT_COUNT] {
            std::array::from_fn(|i| f64::from(f(&joints[i])))
        };
        let raw_position = field(|j| j.position);
        if raw_position.iter().any(|p| !p.is_finite()) {
            log_warn!(
                self.logger.throttle(std::time::Duration::from_secs(5)),
                "non-finite joint position in a JointsData reading, dropped"
            );
            return;
        }

        let calibration = self.calibration.get_or_insert_with(|| {
            let cal = Calibration::from_first_reading(&raw_position);
            let wrapped = cal.wrapped_joints();
            if wrapped.is_empty() {
                log_info!(
                    &self.logger,
                    "first JointsData reading; vendor offsets as they are"
                );
            } else {
                log_warn!(
                    &self.logger,
                    "first JointsData reading; offset moved a full turn for {wrapped:?} \
                     (the vendor's start-up correction)"
                );
            }
            cal
        });

        let out = JointState {
            // Our clock, not the message's: the vendor stamps with its own computer's uptime
            // (seconds since its boot), which robot_state_publisher would put on every leg
            // transform, and no TF lookup on our clock would ever reach them.
            header: Header {
                stamp: stamp(now_ns),
                frame_id: String::new(),
            },
            name: JOINT_NAMES.iter().map(|n| n.to_string()).collect(),
            position: calibration.positions(&raw_position).to_vec(),
            velocity: directed(&field(|j| j.velocity)).to_vec(),
            effort: directed(&field(|j| j.torque)).to_vec(),
        };
        if let Err(e) = self.publisher.publish(out) {
            log_error!(
                self.logger.throttle(std::time::Duration::from_secs(5)),
                "publish joint_states failed: {e}"
            );
            return;
        }
        self.last_publish_ns = Some(now_ns);
    }
}

fn stamp(nsec: i64) -> TimeMsg {
    TimeMsg {
        sec: i32::try_from(nsec.div_euclid(1_000_000_000)).unwrap_or(i32::MAX),
        nanosec: u32::try_from(nsec.rem_euclid(1_000_000_000)).unwrap_or(0),
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut executor = Context::default_from_env()?.create_basic_executor();
    let node = executor.create_node("m20_joint_state")?;

    let input: ReadOnlyParameter<Arc<str>> = node
        .declare_parameter("joints_topic")
        // Absolute: the vendor publishes with no namespace, from its own computers, and a
        // relative name would resolve to /<robot_id>/JOINTS_DATA_10HZ, where nothing ever
        // arrives. The 10 Hz topic, not /JOINTS_DATA (the same message from the same writer at
        // ~190 Hz; the default until 2026-11-24): reading that one meant taking 193 messages a
        // second off the robot's LAN to throw nine in ten away below, 5.5 % of a core on
        // robot01 for 19 Hz of joint_states. The price is leg TF at 10 Hz, half of what
        // robot_state_publisher would pass on. Name /JOINTS_DATA again for a consumer that
        // needs the legs faster than that.
        .default(Arc::<str>::from("/JOINTS_DATA_10HZ"))
        .description("the vendor's drdds/msg/JointsData topic")
        .read_only()?;
    let rate: ReadOnlyParameter<f64> = node
        .declare_parameter("publish_rate")
        // A cap for a fast joints_topic only: the default /JOINTS_DATA_10HZ is under it, so
        // every one of its readings is published. /JOINTS_DATA arrives at ~190 Hz; the leg
        // links' TF is for looking at, and every message here is one robot_state_publisher
        // turns into 16 transforms on /tf. 20 = robot_state_publisher's own publish_frequency
        // default, which caps its moving transforms: at 50 (until 2026-11-24) more than half
        // the joint_states were converted and published only to be dropped there (robot01:
        // 44 Hz in, ~15-20 Hz of /tf out). Raise the two together.
        .default(20.0)
        .range(ParameterRange {
            lower: Some(0.0),
            upper: None,
            step: None,
        })
        .description("joint_states publications per second at most; 0 = one per reading")
        .read_only()?;

    let rate = rate.get();
    let min_period_ns = if rate > 0.0 { (1e9 / rate) as i64 } else { 0 };
    let topic = input.get();

    let bridge = Bridge {
        // Reliable: robot_state_publisher's subscription matches it whatever its own QoS
        publisher: node.create_publisher("joint_states".keep_last(10).reliable())?,
        clock: node.get_clock(),
        logger: node.logger().clone(),
        calibration: None,
        min_period_ns,
        last_publish_ns: None,
    };
    let worker = node.create_worker(bridge);
    // Best effort, keep last: matches the vendor's reliable / transient-local writer, and a
    // reading that comes late is worth less than the next one
    let _subscription = worker.create_subscription(
        (&*topic).sensor_data_qos(),
        |bridge: &mut Bridge, msg: JointsData| bridge.on_reading(msg),
    )?;
    log_info!(
        node.logger(),
        "{topic} -> joint_states at <= {rate} Hz ({} joints)",
        JOINT_COUNT
    );

    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
