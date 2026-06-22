use anyhow::Result;
use rclrs::{MandatoryParameter, Node, QoSProfile};
use std::{path::PathBuf, sync::Arc};

const DEFAULT_DB_PATH: &str = "/tmp/robot_telemetry_server.db";
const DEFAULT_DB_BATCH_SIZE: u32 = 100;
const DEFAULT_DB_FLUSH_INTERVAL_MS: u64 = 1000;

pub(crate) struct DbParameters {
    path: MandatoryParameter<Arc<str>>,
    batch_size: MandatoryParameter<i64>,
    flush_interval_ms: MandatoryParameter<i64>,
}

pub(crate) struct DbConfig {
    pub(crate) path: PathBuf,
    pub(crate) batch_size: u32,
    pub(crate) flush_interval_ms: u64,
}

struct TopicDefaults {
    topic: &'static str,
    frame_id: &'static str,
    child_frame_id: &'static str,
    reliability: &'static str,
    depth: u32,
    durability: &'static str,
}

pub(crate) struct TopicParameters {
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

pub(crate) struct SubscriptionConfig {
    pub(crate) topic: Arc<str>,
    reliability: Arc<str>,
    depth: u32,
    durability: Arc<str>,
    default_reliability: &'static str,
    default_durability: &'static str,
}

pub(crate) struct TelemetryParams {
    pub(crate) db: DbParameters,
    pub(crate) odom: TopicParameters,
    pub(crate) imu: TopicParameters,
    pub(crate) scan: TopicParameters,
    pub(crate) twist: TopicParameters,
    pub(crate) battery: TopicParameters,
    pub(crate) tf: TopicParameters,
    pub(crate) rosout: TopicParameters,
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

impl DbParameters {
    fn declare(node: &Node) -> Result<Self> {
        Ok(Self {
            path: node
                .declare_parameter("db.path")
                .default(Arc::<str>::from(DEFAULT_DB_PATH))
                .mandatory()?,
            batch_size: node
                .declare_parameter("db.batch_size")
                .default(i64::from(DEFAULT_DB_BATCH_SIZE))
                .mandatory()?,
            flush_interval_ms: node
                .declare_parameter("db.flush_interval_ms")
                .default(i64::try_from(DEFAULT_DB_FLUSH_INTERVAL_MS).unwrap())
                .mandatory()?,
        })
    }

    pub(crate) fn config(&self, node: &Node) -> DbConfig {
        let path = self.path.get();
        let path = if path.is_empty() {
            rclrs::log_warn!(
                node.logger(),
                "Invalid db.path '', defaulting to '{}'",
                DEFAULT_DB_PATH
            );
            PathBuf::from(DEFAULT_DB_PATH)
        } else {
            PathBuf::from(path.as_ref())
        };

        DbConfig {
            path,
            batch_size: u32_parameter_or_default(
                node,
                "db.batch_size",
                self.batch_size.get(),
                DEFAULT_DB_BATCH_SIZE,
            ),
            flush_interval_ms: u64_parameter_or_default(
                node,
                "db.flush_interval_ms",
                self.flush_interval_ms.get(),
                DEFAULT_DB_FLUSH_INTERVAL_MS,
            ),
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

    pub(crate) fn subscription_config(&self, node: &Node, label: &str) -> SubscriptionConfig {
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
    pub(crate) fn declare(node: &Node) -> Result<Self> {
        Ok(Self {
            db: DbParameters::declare(node)?,
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
            rosout: TopicParameters::declare(
                node,
                "rosout",
                TopicDefaults::new("/rosout", "", "", "reliable", 1000, "transient_local"),
            )?,
        })
    }
}

fn u32_parameter_or_default(node: &Node, name: &str, value: i64, default: u32) -> u32 {
    match u32::try_from(value) {
        Ok(value) => value,
        Err(_) => {
            rclrs::log_warn!(
                node.logger(),
                "Invalid {} '{}', defaulting to {}",
                name,
                value,
                default
            );
            default
        }
    }
}

fn u64_parameter_or_default(node: &Node, name: &str, value: i64, default: u64) -> u64 {
    match u64::try_from(value) {
        Ok(value) => value,
        Err(_) => {
            rclrs::log_warn!(
                node.logger(),
                "Invalid {} '{}', defaulting to {}",
                name,
                value,
                default
            );
            default
        }
    }
}

fn parameter_name(prefix: &str, suffix: &str) -> Arc<str> {
    let name = format!("{prefix}.{suffix}");
    Arc::<str>::from(name.as_str())
}

pub(crate) fn configure_qos(
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

pub(crate) fn log_subscription_config(node: &Node, label: &str, config: &SubscriptionConfig) {
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
