use anyhow::Result;
use rclrs::*;
use robot_telemetry_server::RobotTelemetryServerNode;

fn main() -> Result<()> {
    let context: Context = Context::default_from_env()?;
    let mut executor: Executor = context.create_basic_executor();
    let _node: RobotTelemetryServerNode =
        RobotTelemetryServerNode::new(&executor, "robot_telemetry_server")?;
    println!("Robot Telemetry Server Node has started.");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
