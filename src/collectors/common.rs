use crate::db::writer::MetricSample;
use std::time::{SystemTime, UNIX_EPOCH};

pub(crate) struct FiniteStats {
    pub(crate) count: usize,
    pub(crate) min: f64,
    pub(crate) max: f64,
    pub(crate) mean: f64,
}

pub(crate) fn timestamp_ns(sec: i32, nanosec: u32) -> i64 {
    i64::from(sec) * 1_000_000_000 + i64::from(nanosec)
}

pub(crate) fn current_timestamp_ns() -> i64 {
    let Ok(duration) = SystemTime::now().duration_since(UNIX_EPOCH) else {
        return 0;
    };

    i64::try_from(duration.as_nanos()).unwrap_or(i64::MAX)
}

pub(crate) fn sample(
    timestamp_ns: i64,
    source_topic: &str,
    metric_name: &str,
    metric_value: f64,
    unit: &str,
) -> MetricSample {
    MetricSample {
        timestamp_ns,
        source_topic: source_topic.to_string(),
        metric_name: metric_name.to_string(),
        metric_value,
        unit: unit.to_string(),
        quality: "ok".to_string(),
    }
}

pub(crate) fn push_finite_sample(
    samples: &mut Vec<MetricSample>,
    timestamp_ns: i64,
    source_topic: &str,
    metric_name: &str,
    metric_value: f64,
    unit: &str,
) {
    if metric_value.is_finite() {
        samples.push(sample(
            timestamp_ns,
            source_topic,
            metric_name,
            metric_value,
            unit,
        ));
    }
}

pub(crate) fn push_count_sample(
    samples: &mut Vec<MetricSample>,
    timestamp_ns: i64,
    source_topic: &str,
    metric_name: &str,
    count: usize,
) {
    samples.push(sample(
        timestamp_ns,
        source_topic,
        metric_name,
        count as f64,
        "count",
    ));
}

pub(crate) fn finite_stats(values: impl IntoIterator<Item = f64>) -> Option<FiniteStats> {
    let mut count = 0usize;
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    let mut sum = 0.0f64;

    for value in values {
        if !value.is_finite() {
            continue;
        }

        count += 1;
        min = min.min(value);
        max = max.max(value);
        sum += value;
    }

    if count == 0 {
        None
    } else {
        Some(FiniteStats {
            count,
            min,
            max,
            mean: sum / count as f64,
        })
    }
}
