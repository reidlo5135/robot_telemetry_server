use sensor_msgs::msg::LaserScan;

use crate::collectors::common::{
    finite_stats, push_count_sample, push_finite_sample, timestamp_ns,
};
use crate::db::writer::MetricSample;

pub(crate) fn extract_scan_metrics(msg: &LaserScan, source_topic: &str) -> Vec<MetricSample> {
    let timestamp_ns = timestamp_ns(msg.header.stamp.sec, msg.header.stamp.nanosec);
    let mut samples = Vec::with_capacity(18);

    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "scan_angle_min",
        f64::from(msg.angle_min),
        "rad",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "scan_angle_max",
        f64::from(msg.angle_max),
        "rad",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "scan_angle_increment",
        f64::from(msg.angle_increment),
        "rad",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "scan_time_increment",
        f64::from(msg.time_increment),
        "s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "scan_time",
        f64::from(msg.scan_time),
        "s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "scan_range_min_limit",
        f64::from(msg.range_min),
        "m",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "scan_range_max_limit",
        f64::from(msg.range_max),
        "m",
    );

    push_count_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "scan_range_count",
        msg.ranges.len(),
    );

    let valid_ranges =
        msg.ranges.iter().copied().filter(|range| {
            range.is_finite() && *range >= msg.range_min && *range <= msg.range_max
        });
    if let Some(range_stats) = finite_stats(valid_ranges.map(f64::from)) {
        push_count_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_valid_range_count",
            range_stats.count,
        );
        push_count_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_invalid_range_count",
            msg.ranges.len().saturating_sub(range_stats.count),
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_range_min",
            range_stats.min,
            "m",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_range_max",
            range_stats.max,
            "m",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_range_mean",
            range_stats.mean,
            "m",
        );
    } else {
        push_count_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_valid_range_count",
            0,
        );
        push_count_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_invalid_range_count",
            msg.ranges.len(),
        );
    }

    if let Some(intensity_stats) = finite_stats(msg.intensities.iter().copied().map(f64::from)) {
        push_count_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_intensity_count",
            intensity_stats.count,
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_intensity_min",
            intensity_stats.min,
            "intensity",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_intensity_max",
            intensity_stats.max,
            "intensity",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "scan_intensity_mean",
            intensity_stats.mean,
            "intensity",
        );
    }

    samples
}
