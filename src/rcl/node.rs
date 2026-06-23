use anyhow::Result;
use rclrs::*;
use std::sync::Arc;

use crate::collectors::battery::extract_battery_metrics;
use crate::collectors::common::current_timestamp_ns;
use crate::collectors::imu::extract_imu_metrics;
use crate::collectors::odom::extract_odom_metrics;
use crate::collectors::rosout::{extract_rosout_event, extract_rosout_metrics};
use crate::collectors::scan::extract_scan_metrics;
use crate::collectors::tf::extract_tf_metrics;
use crate::collectors::twist::extract_twist_metrics;
use crate::db::writer::{MetricSample, SqliteWriterHandle};
use crate::rcl::params::{TelemetryParams, configure_qos, log_subscription_config};
use crate::state::TelemetryState;

pub struct RobotTelemetryServerNode {
    #[allow(dead_code)]
    node: Node,

    // Keep parameter handles alive; rclrs undeclares parameters when handles are dropped.
    #[allow(dead_code)]
    params: TelemetryParams,

    #[allow(dead_code)]
    db_writer: SqliteWriterHandle,

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

    #[allow(dead_code)]
    rosout_subscription: WorkerSubscription<rcl_interfaces::msg::Log, TelemetryState>,
}

impl RobotTelemetryServerNode {
    pub fn new(executor: &Executor, name: &str) -> Result<Self> {
        let node: Arc<NodeState> = executor.create_node(name)?;

        let params = TelemetryParams::declare(&node)?;
        let db_config = params.db.config(&node);
        rclrs::log_info!(
            node.logger(),
            "Writing telemetry database to '{}' (batch_size={}, flush_interval_ms={})",
            db_config.path.display(),
            db_config.batch_size,
            db_config.flush_interval_ms
        );
        let db_writer = SqliteWriterHandle::spawn(db_config.path.clone())?;

        let worker: Arc<WorkerState<TelemetryState>> = node.create_worker(TelemetryState {
            node: Arc::clone(&node),
            db_writer: db_writer.clone(),
        });

        let odom_config = params.odom.subscription_config(&node, "odom");
        log_subscription_config(&node, "odom", &odom_config);
        let odom_topic_name = odom_config.topic.to_string();
        let mut odom_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(odom_config.topic.as_ref());
        odom_subscription_opts.qos =
            configure_qos(&node, "odom", odom_subscription_opts.qos, &odom_config);

        let odom_subscription: Arc<
            SubscriptionState<nav_msgs::msg::Odometry, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            odom_subscription_opts,
            move |state: &mut TelemetryState, msg: nav_msgs::msg::Odometry| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /odom message linear.x={:.3}, angular.z={:.3}",
                    msg.twist.twist.linear.x,
                    msg.twist.twist.angular.z
                );

                queue_metrics(
                    state,
                    "odom",
                    extract_odom_metrics(&msg, odom_topic_name.as_str()),
                );
            },
        )?;

        let imu_config = params.imu.subscription_config(&node, "imu");
        log_subscription_config(&node, "imu", &imu_config);
        let mut imu_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(imu_config.topic.as_ref());
        imu_subscription_opts.qos =
            configure_qos(&node, "imu", imu_subscription_opts.qos, &imu_config);
        let imu_topic_name = imu_config.topic.to_string();

        let imu_subscription: Arc<
            SubscriptionState<sensor_msgs::msg::Imu, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            imu_subscription_opts,
            move |state: &mut TelemetryState, msg: sensor_msgs::msg::Imu| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /imu message linear_acceleration.x={:.3}, angular_velocity.z={:.3}",
                    msg.linear_acceleration.x,
                    msg.angular_velocity.z
                );

                queue_metrics(
                    state,
                    "imu",
                    extract_imu_metrics(&msg, imu_topic_name.as_str()),
                );
            },
        )?;

        let scan_config = params.scan.subscription_config(&node, "scan");
        log_subscription_config(&node, "scan", &scan_config);
        let mut scan_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(scan_config.topic.as_ref());
        scan_subscription_opts.qos =
            configure_qos(&node, "scan", scan_subscription_opts.qos, &scan_config);
        let scan_topic_name = scan_config.topic.to_string();

        let scan_subscription: Arc<
            SubscriptionState<sensor_msgs::msg::LaserScan, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            scan_subscription_opts,
            move |state: &mut TelemetryState, msg: sensor_msgs::msg::LaserScan| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /scan message with {} ranges",
                    msg.ranges.len()
                );

                queue_metrics(
                    state,
                    "scan",
                    extract_scan_metrics(&msg, scan_topic_name.as_str()),
                );
            },
        )?;

        let twist_config = params.twist.subscription_config(&node, "twist");
        log_subscription_config(&node, "twist", &twist_config);
        let mut twist_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(twist_config.topic.as_ref());
        twist_subscription_opts.qos =
            configure_qos(&node, "twist", twist_subscription_opts.qos, &twist_config);
        let twist_topic_name = twist_config.topic.to_string();

        let twist_subscription: Arc<
            SubscriptionState<geometry_msgs::msg::Twist, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            twist_subscription_opts,
            move |state: &mut TelemetryState, msg: geometry_msgs::msg::Twist| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /cmd_vel message linear.x={:.3}, angular.z={:.3}",
                    msg.linear.x,
                    msg.angular.z
                );

                queue_metrics(
                    state,
                    "twist",
                    extract_twist_metrics(&msg, twist_topic_name.as_str(), current_timestamp_ns()),
                );
            },
        )?;

        let battery_config = params.battery.subscription_config(&node, "battery");
        log_subscription_config(&node, "battery", &battery_config);
        let mut battery_state_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(battery_config.topic.as_ref());
        battery_state_subscription_opts.qos = configure_qos(
            &node,
            "battery",
            battery_state_subscription_opts.qos,
            &battery_config,
        );
        let battery_topic_name = battery_config.topic.to_string();

        let battery_state_subscription: Arc<
            SubscriptionState<sensor_msgs::msg::BatteryState, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            battery_state_subscription_opts,
            move |state: &mut TelemetryState, msg: sensor_msgs::msg::BatteryState| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /battery_state message voltage={:.3}, current={:.3}",
                    msg.voltage,
                    msg.current
                );

                queue_metrics(
                    state,
                    "battery",
                    extract_battery_metrics(&msg, battery_topic_name.as_str()),
                );
            },
        )?;

        let tf_config = params.tf.subscription_config(&node, "tf");
        log_subscription_config(&node, "tf", &tf_config);
        let mut tf_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(tf_config.topic.as_ref());
        tf_subscription_opts.qos = configure_qos(&node, "tf", tf_subscription_opts.qos, &tf_config);
        let tf_topic_name = tf_config.topic.to_string();

        let tf_subscription: Arc<
            SubscriptionState<tf2_msgs::msg::TFMessage, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            tf_subscription_opts,
            move |state: &mut TelemetryState, msg: tf2_msgs::msg::TFMessage| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /tf message with {} transforms",
                    msg.transforms.len()
                );

                queue_metrics(
                    state,
                    "tf",
                    extract_tf_metrics(&msg, tf_topic_name.as_str()),
                );
            },
        )?;

        let rosout_config = params.rosout.subscription_config(&node, "rosout");
        log_subscription_config(&node, "rosout", &rosout_config);
        let rosout_topic_name = rosout_config.topic.to_string();
        let mut rosout_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(rosout_config.topic.as_ref());
        rosout_subscription_opts.qos = configure_qos(
            &node,
            "rosout",
            rosout_subscription_opts.qos,
            &rosout_config,
        );

        let rosout_subscription: Arc<
            SubscriptionState<rcl_interfaces::msg::Log, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            rosout_subscription_opts,
            move |state: &mut TelemetryState, msg: rcl_interfaces::msg::Log| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /rosout message from {}: '{}'",
                    msg.name,
                    msg.msg
                );

                queue_metrics(
                    state,
                    "rosout",
                    extract_rosout_metrics(&msg, rosout_topic_name.as_str()),
                );

                let event = extract_rosout_event(&msg, rosout_topic_name.as_str());

                if let Err(error) = state.db_writer.write_event(event) {
                    rclrs::log_warn!(
                        state.node.logger(),
                        "Failed to queue rosout event for sqlite writer: {}",
                        error
                    );
                }
            },
        )?;

        Ok(Self {
            node,
            params,
            db_writer,
            odom_subscription,
            imu_subscription,
            scan_subscription,
            twist_subscription,
            battery_state_subscription,
            tf_subscription,
            rosout_subscription,
        })
    }
}

fn queue_metrics(state: &TelemetryState, label: &str, samples: Vec<MetricSample>) {
    for sample in samples {
        if let Err(error) = state.db_writer.write_metric(sample) {
            rclrs::log_warn!(
                state.node.logger(),
                "Failed to queue {} metric for sqlite writer: {}",
                label,
                error
            );
        }
    }
}
