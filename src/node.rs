use anyhow::Result;
use rclrs::*;
use std::sync::Arc;

use crate::collectors::odom::extract_odom_metric;
use crate::state::TelemetryState;

pub struct RobotTelemetryServerNode {
    #[allow(dead_code)]
    node: Node,

    #[allow(dead_code)]
    odom_subscription: WorkerSubscription<nav_msgs::msg::Odometry, TelemetryState>,

    #[allow(dead_code)]
    imu_subscription: WorkerSubscription<sensor_msgs::msg::Imu, TelemetryState>,

    #[allow(dead_code)]
    scan_subscription: WorkerSubscription<sensor_msgs::msg::LaserScan, TelemetryState>,

    #[allow(dead_code)]
    twist_subscription: WorkerSubscription<geometry_msgs::msg::Twist, TelemetryState>,

    #[allow(dead_code)]
    battery_state_subscription: WorkerSubscription<sensor_msgs::msg::BatteryState, TelemetryState>,

    #[allow(dead_code)]
    tf_subscription: WorkerSubscription<tf2_msgs::msg::TFMessage, TelemetryState>,
}

impl RobotTelemetryServerNode {
    pub fn new(executor: &Executor, name: &str) -> Result<Self, RclrsError> {
        let node: Arc<NodeState> = executor.create_node(name)?;

        let worker: Arc<WorkerState<TelemetryState>> = node.create_worker(TelemetryState {
            node: Arc::clone(&node),
        });

        let mut odom_subscription_opts: SubscriptionOptions<'_> = SubscriptionOptions::new("/odom");
        odom_subscription_opts.qos = odom_subscription_opts.qos.reliable().keep_last(10);

        let odom_subscription: Arc<
            SubscriptionState<nav_msgs::msg::Odometry, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            odom_subscription_opts,
            |state: &mut TelemetryState, msg: nav_msgs::msg::Odometry| {
                let (linear_x, angular_z) = extract_odom_metric(&msg);
                rclrs::log_info!(
                    state.node.logger(),
                    "Received /odom message linear.x={:.3}, angular.z={:.3}",
                    linear_x,
                    angular_z
                );
            },
        )?;

        let mut imu_subscription_opts: SubscriptionOptions<'_> = SubscriptionOptions::new("/imu");
        imu_subscription_opts.qos = imu_subscription_opts.qos.reliable().keep_last(10);

        let imu_subscription: Arc<
            SubscriptionState<sensor_msgs::msg::Imu, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            imu_subscription_opts,
            |state: &mut TelemetryState, msg: sensor_msgs::msg::Imu| {
                rclrs::log_info!(
                    state.node.logger(),
                    "Received /imu message linear_acceleration.x={:.3}, angular_velocity.z={:.3}",
                    msg.linear_acceleration.x,
                    msg.angular_velocity.z
                );
            },
        )?;

        let mut scan_subscription_opts: SubscriptionOptions<'_> = SubscriptionOptions::new("/scan");
        scan_subscription_opts.qos = scan_subscription_opts.qos.best_effort().keep_last(5);

        let scan_subscription: Arc<
            SubscriptionState<sensor_msgs::msg::LaserScan, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            scan_subscription_opts,
            |state: &mut TelemetryState, msg: sensor_msgs::msg::LaserScan| {
                rclrs::log_info!(
                    state.node.logger(),
                    "Received /scan message with {} ranges",
                    msg.ranges.len()
                );
            },
        )?;

        let mut twist_subscription_opts: SubscriptionOptions<'_> = SubscriptionOptions::new("/cmd_vel");
        twist_subscription_opts.qos = twist_subscription_opts.qos.reliable().keep_last(10);

        let twist_subscription: Arc<
            SubscriptionState<geometry_msgs::msg::Twist, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            twist_subscription_opts,
            |state: &mut TelemetryState, msg: geometry_msgs::msg::Twist| {
                rclrs::log_info!(
                    state.node.logger(),
                    "Received /cmd_vel message linear.x={:.3}, angular.z={:.3}",
                    msg.linear.x,
                    msg.angular.z
                );
            },
        )?;

        let mut battery_state_subscription_opts: SubscriptionOptions<'_> = SubscriptionOptions::new("/battery_state");
        battery_state_subscription_opts.qos = battery_state_subscription_opts.qos.reliable().keep_last(10);

        let battery_state_subscription: Arc<
            SubscriptionState<sensor_msgs::msg::BatteryState, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            battery_state_subscription_opts,
            |state: &mut TelemetryState, msg: sensor_msgs::msg::BatteryState| {
                rclrs::log_info!(
                    state.node.logger(),
                    "Received /battery_state message voltage={:.3}, current={:.3}",
                    msg.voltage,
                    msg.current
                );
            },
        )?;

        let mut tf_subscription_opts: SubscriptionOptions<'_> = SubscriptionOptions::new("/tf");
        tf_subscription_opts.qos = tf_subscription_opts.qos.reliable().keep_last(100);

        let tf_subscription: Arc<
            SubscriptionState<tf2_msgs::msg::TFMessage, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            tf_subscription_opts,
            |state: &mut TelemetryState, msg: tf2_msgs::msg::TFMessage| {
                rclrs::log_info!(
                    state.node.logger(),
                    "Received /tf message with {} transforms",
                    msg.transforms.len()
                );
            },
        )?;

        Ok(Self {
            node,
            odom_subscription,
            imu_subscription,
            scan_subscription,
            twist_subscription,
            battery_state_subscription,
            tf_subscription,
        })
    }
}
