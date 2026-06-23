use tf2_msgs::msg::TFMessage;

use crate::collectors::common::{
    current_timestamp_ns, finite_stats, push_count_sample, push_finite_sample, timestamp_ns,
};
use crate::db::writer::MetricSample;

pub(crate) fn extract_tf_metrics(msg: &TFMessage, source_topic: &str) -> Vec<MetricSample> {
    let timestamp_ns = msg
        .transforms
        .first()
        .map(|transform| timestamp_ns(transform.header.stamp.sec, transform.header.stamp.nanosec))
        .unwrap_or_else(current_timestamp_ns);
    let mut samples = Vec::with_capacity(8);

    push_count_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "tf_transform_count",
        msg.transforms.len(),
    );

    let translation_norms = msg.transforms.iter().map(|transform| {
        let translation = &transform.transform.translation;
        (translation.x.powi(2) + translation.y.powi(2) + translation.z.powi(2)).sqrt()
    });
    if let Some(translation_stats) = finite_stats(translation_norms) {
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "tf_translation_norm_min",
            translation_stats.min,
            "m",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "tf_translation_norm_max",
            translation_stats.max,
            "m",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "tf_translation_norm_mean",
            translation_stats.mean,
            "m",
        );
    }

    let rotation_angles = msg.transforms.iter().filter_map(|transform| {
        let rotation = &transform.transform.rotation;
        let quaternion_norm =
            (rotation.x.powi(2) + rotation.y.powi(2) + rotation.z.powi(2) + rotation.w.powi(2))
                .sqrt();

        if !quaternion_norm.is_finite() || quaternion_norm == 0.0 {
            return None;
        }

        let normalized_w = (rotation.w / quaternion_norm).clamp(-1.0, 1.0);
        Some(2.0 * normalized_w.acos())
    });
    if let Some(rotation_stats) = finite_stats(rotation_angles) {
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "tf_rotation_angle_min",
            rotation_stats.min,
            "rad",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "tf_rotation_angle_max",
            rotation_stats.max,
            "rad",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "tf_rotation_angle_mean",
            rotation_stats.mean,
            "rad",
        );
    }

    samples
}
