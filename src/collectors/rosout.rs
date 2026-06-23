use rcl_interfaces::msg::Log;

use crate::collectors::common::{sample, timestamp_ns};
use crate::db::writer::{EventLog, MetricSample};

pub(crate) fn extract_rosout_event(msg: &Log, source_topic: &str) -> EventLog {
    EventLog {
        timestamp_ns: timestamp_ns(msg.stamp.sec, msg.stamp.nanosec),
        source_topic: source_topic.to_string(),
        logger_name: msg.name.clone(),
        severity: i64::from(msg.level),
        message: msg.msg.clone(),
        file: optional_string(msg.file.clone()),
        function_name: optional_string(msg.function.clone()),
        line: Some(i64::from(msg.line)),
    }
}

pub(crate) fn extract_rosout_metrics(msg: &Log, source_topic: &str) -> Vec<MetricSample> {
    vec![sample(
        timestamp_ns(msg.stamp.sec, msg.stamp.nanosec),
        source_topic,
        "rosout_severity",
        f64::from(msg.level),
        "level",
    )]
}

fn optional_string(value: String) -> Option<String> {
    if value.is_empty() { None } else { Some(value) }
}
