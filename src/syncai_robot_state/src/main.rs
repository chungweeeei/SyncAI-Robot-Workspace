use anyhow::{Error, Result};
use rclrs::*;
use std::time::Duration;

struct RobotStateNode {
    node: Node,                  
    publisher: Publisher<syncai_common::msg::RobotState>,
}

impl RobotStateNode {

    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("syncai_robot_state")?;
        let publisher = node.create_publisher::<syncai_common::msg::RobotState>("robot_state")?;
        Ok(Self { node, publisher })
    }

    /// 一個方法：&self 就是其他語言的 this / self。
    fn publish_state(&self, count: u32) {
        let msg = syncai_common::msg::RobotState {
            timestamp: rclrs::Clock::ros_time().now().to_msg(),
            mode: syncai_common::msg::RobotMode::Idle,
            localization_status: syncai_common::msg::RobotLocalizationStatus {
                position: syncai_common::msg::RobotPosition {
                    x: 0.0,
                    y: 0.0,
                    theta: 0.0,
                },
                velocity: 0.0,
            },
            network_status: syncai_common::msg::RobotNetworkStatus {
                wifi_info: "testing..."
            },
            battery_status: syncai_common::msg::RobotBatteryStatus {
                battery_percentage: 62.0
            },
        };
        self.publisher.publish(msg).ok();
    }
}

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let robot = RobotStateNode::new(&executor)?;

    // spin：持續處理 timer/訂閱等事件，直到 Ctrl-C。
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
