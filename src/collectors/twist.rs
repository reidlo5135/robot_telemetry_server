use geometry_msgs::msg::Twist;

use crate::collectors::common::push_finite_sample;
use crate::db::writer::MetricSample;

pub(crate) fn extract_twist_metrics(
    msg: &Twist,
    source_topic: &str,
    timestamp_ns: i64,
) -> Vec<MetricSample> {
    let mut samples = Vec::with_capacity(6);

    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "cmd_vel_linear_x",
        msg.linear.x,
        "m/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "cmd_vel_linear_y",
        msg.linear.y,
        "m/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "cmd_vel_linear_z",
        msg.linear.z,
        "m/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "cmd_vel_angular_x",
        msg.angular.x,
        "rad/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "cmd_vel_angular_y",
        msg.angular.y,
        "rad/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "cmd_vel_angular_z",
        msg.angular.z,
        "rad/s",
    );

    samples
}
