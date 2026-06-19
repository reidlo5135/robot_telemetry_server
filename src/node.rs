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
    scan_subscription: WorkerSubscription<sensor_msgs::msg::LaserScan, TelemetryState>,
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

        Ok(Self {
            node,
            odom_subscription,
            scan_subscription,
        })
    }
}
