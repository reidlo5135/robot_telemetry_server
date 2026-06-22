use anyhow::Result;
use rclrs::*;
use std::sync::Arc;

use crate::collectors::odom::extract_odom_metric;
use crate::state::TelemetryState;

struct TopicDefaults {
    topic: &'static str,
    frame_id: &'static str,
    child_frame_id: &'static str,
    reliability: &'static str,
    depth: u32,
    durability: &'static str,
}

struct TopicParameters {
    topic: MandatoryParameter<Arc<str>>,
    #[allow(dead_code)]
    frame_id: MandatoryParameter<Arc<str>>,
    #[allow(dead_code)]
    child_frame_id: MandatoryParameter<Arc<str>>,
    reliability: MandatoryParameter<Arc<str>>,
    depth: MandatoryParameter<i64>,
    durability: MandatoryParameter<Arc<str>>,
    default_reliability: &'static str,
    default_depth: u32,
    default_durability: &'static str,
}

struct SubscriptionConfig {
    topic: Arc<str>,
    reliability: Arc<str>,
    depth: u32,
    durability: Arc<str>,
    default_reliability: &'static str,
    default_durability: &'static str,
}

struct TelemetryParams {
    odom: TopicParameters,
    imu: TopicParameters,
    scan: TopicParameters,
    twist: TopicParameters,
    battery: TopicParameters,
    tf: TopicParameters,
}

pub struct RobotTelemetryServerNode {
    #[allow(dead_code)]
    node: Node,

    // Keep parameter handles alive; rclrs undeclares parameters when handles are dropped.
    #[allow(dead_code)]
    params: TelemetryParams,

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

impl TopicDefaults {
    const fn new(
        topic: &'static str,
        frame_id: &'static str,
        child_frame_id: &'static str,
        reliability: &'static str,
        depth: u32,
        durability: &'static str,
    ) -> Self {
        Self {
            topic,
            frame_id,
            child_frame_id,
            reliability,
            depth,
            durability,
        }
    }
}

impl TopicParameters {
    fn declare(node: &Node, prefix: &str, defaults: TopicDefaults) -> Result<Self> {
        Ok(Self {
            topic: node
                .declare_parameter(parameter_name(prefix, "topic"))
                .default(Arc::<str>::from(defaults.topic))
                .mandatory()?,
            frame_id: node
                .declare_parameter(parameter_name(prefix, "frame_id"))
                .default(Arc::<str>::from(defaults.frame_id))
                .mandatory()?,
            child_frame_id: node
                .declare_parameter(parameter_name(prefix, "child_frame_id"))
                .default(Arc::<str>::from(defaults.child_frame_id))
                .mandatory()?,
            reliability: node
                .declare_parameter(parameter_name(prefix, "qos.reliability"))
                .default(Arc::<str>::from(defaults.reliability))
                .mandatory()?,
            depth: node
                .declare_parameter(parameter_name(prefix, "qos.depth"))
                .default(i64::from(defaults.depth))
                .mandatory()?,
            durability: node
                .declare_parameter(parameter_name(prefix, "qos.durability"))
                .default(Arc::<str>::from(defaults.durability))
                .mandatory()?,
            default_reliability: defaults.reliability,
            default_depth: defaults.depth,
            default_durability: defaults.durability,
        })
    }

    fn subscription_config(&self, node: &Node, label: &str) -> SubscriptionConfig {
        let raw_depth = self.depth.get();
        let depth = match u32::try_from(raw_depth) {
            Ok(depth) => depth,
            Err(_) => {
                rclrs::log_warn!(
                    node.logger(),
                    "Invalid {} qos.depth '{}', defaulting to {}",
                    label,
                    raw_depth,
                    self.default_depth
                );
                self.default_depth
            }
        };

        SubscriptionConfig {
            topic: self.topic.get(),
            reliability: self.reliability.get(),
            depth,
            durability: self.durability.get(),
            default_reliability: self.default_reliability,
            default_durability: self.default_durability,
        }
    }
}

impl TelemetryParams {
    fn declare(node: &Node) -> Result<Self> {
        Ok(Self {
            odom: TopicParameters::declare(
                node,
                "odom",
                TopicDefaults::new(
                    "/odom",
                    "odom",
                    "base_footprint",
                    "reliable",
                    10,
                    "volatile",
                ),
            )?,
            imu: TopicParameters::declare(
                node,
                "imu",
                TopicDefaults::new("/imu", "imu_link", "", "reliable", 10, "volatile"),
            )?,
            scan: TopicParameters::declare(
                node,
                "scan",
                TopicDefaults::new("/scan", "base_scan", "", "best_effort", 5, "volatile"),
            )?,
            twist: TopicParameters::declare(
                node,
                "twist",
                TopicDefaults::new("/cmd_vel", "", "", "reliable", 10, "volatile"),
            )?,
            battery: TopicParameters::declare(
                node,
                "battery",
                TopicDefaults::new("/battery_state", "", "", "reliable", 10, "volatile"),
            )?,
            tf: TopicParameters::declare(
                node,
                "tf",
                TopicDefaults::new("/tf", "", "", "reliable", 100, "volatile"),
            )?,
        })
    }
}

fn parameter_name(prefix: &str, suffix: &str) -> Arc<str> {
    let name = format!("{prefix}.{suffix}");
    Arc::<str>::from(name.as_str())
}

fn configure_qos(
    node: &Node,
    label: &str,
    base_qos: QoSProfile,
    config: &SubscriptionConfig,
) -> QoSProfile {
    let qos = match config.reliability.as_ref() {
        "reliable" => base_qos.reliable(),
        "best_effort" => base_qos.best_effort(),
        invalid => {
            rclrs::log_warn!(
                node.logger(),
                "Invalid {} qos.reliability '{}', defaulting to '{}'",
                label,
                invalid,
                config.default_reliability
            );
            match config.default_reliability {
                "best_effort" => base_qos.best_effort(),
                _ => base_qos.reliable(),
            }
        }
    };

    let qos = match config.durability.as_ref() {
        "volatile" => qos.volatile(),
        "transient_local" => qos.transient_local(),
        invalid => {
            rclrs::log_warn!(
                node.logger(),
                "Invalid {} qos.durability '{}', defaulting to '{}'",
                label,
                invalid,
                config.default_durability
            );
            match config.default_durability {
                "transient_local" => qos.transient_local(),
                _ => qos.volatile(),
            }
        }
    };

    qos.keep_last(config.depth)
}

fn log_subscription_config(node: &Node, label: &str, config: &SubscriptionConfig) {
    rclrs::log_info!(
        node.logger(),
        "Configured {} subscription with topic='{}', reliability='{}', durability='{}', depth={}",
        label,
        config.topic,
        config.reliability,
        config.durability,
        config.depth
    );
}

impl RobotTelemetryServerNode {
    pub fn new(executor: &Executor, name: &str) -> Result<Self> {
        let node: Arc<NodeState> = executor.create_node(name)?;

        let worker: Arc<WorkerState<TelemetryState>> = node.create_worker(TelemetryState {
            node: Arc::clone(&node),
        });

        let params = TelemetryParams::declare(&node)?;

        let odom_config = params.odom.subscription_config(&node, "odom");
        log_subscription_config(&node, "odom", &odom_config);
        let mut odom_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(odom_config.topic.as_ref());
        odom_subscription_opts.qos =
            configure_qos(&node, "odom", odom_subscription_opts.qos, &odom_config);

        let odom_subscription: Arc<
            SubscriptionState<nav_msgs::msg::Odometry, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            odom_subscription_opts,
            |state: &mut TelemetryState, msg: nav_msgs::msg::Odometry| {
                let (linear_x, angular_z) = extract_odom_metric(&msg);
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /odom message linear.x={:.3}, angular.z={:.3}",
                    linear_x,
                    angular_z
                );
            },
        )?;

        let imu_config = params.imu.subscription_config(&node, "imu");
        log_subscription_config(&node, "imu", &imu_config);
        let mut imu_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(imu_config.topic.as_ref());
        imu_subscription_opts.qos =
            configure_qos(&node, "imu", imu_subscription_opts.qos, &imu_config);

        let imu_subscription: Arc<
            SubscriptionState<sensor_msgs::msg::Imu, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            imu_subscription_opts,
            |state: &mut TelemetryState, msg: sensor_msgs::msg::Imu| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /imu message linear_acceleration.x={:.3}, angular_velocity.z={:.3}",
                    msg.linear_acceleration.x,
                    msg.angular_velocity.z
                );
            },
        )?;

        let scan_config = params.scan.subscription_config(&node, "scan");
        log_subscription_config(&node, "scan", &scan_config);
        let mut scan_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(scan_config.topic.as_ref());
        scan_subscription_opts.qos =
            configure_qos(&node, "scan", scan_subscription_opts.qos, &scan_config);

        let scan_subscription: Arc<
            SubscriptionState<sensor_msgs::msg::LaserScan, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            scan_subscription_opts,
            |state: &mut TelemetryState, msg: sensor_msgs::msg::LaserScan| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /scan message with {} ranges",
                    msg.ranges.len()
                );
            },
        )?;

        let twist_config = params.twist.subscription_config(&node, "twist");
        log_subscription_config(&node, "twist", &twist_config);
        let mut twist_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(twist_config.topic.as_ref());
        twist_subscription_opts.qos =
            configure_qos(&node, "twist", twist_subscription_opts.qos, &twist_config);

        let twist_subscription: Arc<
            SubscriptionState<geometry_msgs::msg::Twist, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            twist_subscription_opts,
            |state: &mut TelemetryState, msg: geometry_msgs::msg::Twist| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /cmd_vel message linear.x={:.3}, angular.z={:.3}",
                    msg.linear.x,
                    msg.angular.z
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

        let battery_state_subscription: Arc<
            SubscriptionState<sensor_msgs::msg::BatteryState, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            battery_state_subscription_opts,
            |state: &mut TelemetryState, msg: sensor_msgs::msg::BatteryState| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /battery_state message voltage={:.3}, current={:.3}",
                    msg.voltage,
                    msg.current
                );
            },
        )?;

        let tf_config = params.tf.subscription_config(&node, "tf");
        log_subscription_config(&node, "tf", &tf_config);
        let mut tf_subscription_opts: SubscriptionOptions<'_> =
            SubscriptionOptions::new(tf_config.topic.as_ref());
        tf_subscription_opts.qos = configure_qos(&node, "tf", tf_subscription_opts.qos, &tf_config);

        let tf_subscription: Arc<
            SubscriptionState<tf2_msgs::msg::TFMessage, Arc<WorkerState<TelemetryState>>>,
        > = worker.create_subscription(
            tf_subscription_opts,
            |state: &mut TelemetryState, msg: tf2_msgs::msg::TFMessage| {
                rclrs::log_debug!(
                    state.node.logger(),
                    "Received /tf message with {} transforms",
                    msg.transforms.len()
                );
            },
        )?;

        Ok(Self {
            node,
            params,
            odom_subscription,
            imu_subscription,
            scan_subscription,
            twist_subscription,
            battery_state_subscription,
            tf_subscription,
        })
    }
}
