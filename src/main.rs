use anyhow::Result;
use rclrs::*;

mod collectors;
mod node;
mod state;

fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor: Executor = context.create_basic_executor();
    let _node: node::RobotTelemetryServerNode =
        node::RobotTelemetryServerNode::new(&executor, "robot_telemetry_server")?;
    println!("Robot Telemetry Server Node has started.");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
