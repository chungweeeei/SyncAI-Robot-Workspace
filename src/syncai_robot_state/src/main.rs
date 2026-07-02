use anyhow::{Error, Result};
use rclrs::*;
use std::time::Duration;

/// 我們的「node 物件」。struct 只描述「有哪些資料」。
struct RobotStateNode {
    node: Node,                                  // ROS node 本體 (其實是個 Arc handle，可便宜地複製)
    publisher: Publisher<std_msgs::msg::String>, // 存起來，之後重複用
}

impl RobotStateNode {
    /// 建構子：建立 node 與 publisher，打包成 Self 回傳。
    fn new(executor: &Executor) -> Result<Self, RclrsError> {
        let node = executor.create_node("syncai_robot_state")?;
        let publisher = node.create_publisher::<std_msgs::msg::String>("chatter")?;
        Ok(Self { node, publisher })
    }

    /// 一個方法：&self 就是其他語言的 this / self。
    fn publish_state(&self, count: u32) {
        let msg = std_msgs::msg::String {
            data: format!("Hello from rclrs! {count}"),
        };
        println!("Publishing: [{}]", msg.data);
        self.publisher.publish(msg).ok(); // .ok() = 忽略可能的錯誤
    }
}

fn main() -> Result<(), Error> {
    let mut executor = Context::default_from_env()?.create_basic_executor();

    let robot = RobotStateNode::new(&executor)?; // 建立我們的 node 物件

    // count 是「會變動的狀態」。rclrs 用 worker 安全保管它，
    // callback 每次會拿到 &mut count 讓你修改（Rust 執行緒安全的設計）。
    let worker = robot.node.create_worker::<u32>(0);

    // 每 500ms 觸發，取代原本手寫的 sleep 迴圈。
    let _timer = worker.create_timer_repeating(
        Duration::from_millis(500),
        move |count: &mut u32| {
            *count += 1;
            robot.publish_state(*count);
        },
    )?;

    // spin：持續處理 timer/訂閱等事件，直到 Ctrl-C。
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
