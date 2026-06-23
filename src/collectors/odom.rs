use nav_msgs::msg::Odometry;

use crate::collectors::common::{push_finite_sample, timestamp_ns};
use crate::db::writer::MetricSample;

pub(crate) fn extract_odom_metrics(msg: &Odometry, source_topic: &str) -> Vec<MetricSample> {
    let timestamp_ns = timestamp_ns(msg.header.stamp.sec, msg.header.stamp.nanosec);
    let mut samples = Vec::with_capacity(13);

    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_position_x",
        msg.pose.pose.position.x,
        "m",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_position_y",
        msg.pose.pose.position.y,
        "m",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_position_z",
        msg.pose.pose.position.z,
        "m",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_orientation_x",
        msg.pose.pose.orientation.x,
        "quaternion",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_orientation_y",
        msg.pose.pose.orientation.y,
        "quaternion",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_orientation_z",
        msg.pose.pose.orientation.z,
        "quaternion",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_orientation_w",
        msg.pose.pose.orientation.w,
        "quaternion",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_linear_x",
        msg.twist.twist.linear.x,
        "m/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_linear_y",
        msg.twist.twist.linear.y,
        "m/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_linear_z",
        msg.twist.twist.linear.z,
        "m/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_angular_x",
        msg.twist.twist.angular.x,
        "rad/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_angular_y",
        msg.twist.twist.angular.y,
        "rad/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "odom_angular_z",
        msg.twist.twist.angular.z,
        "rad/s",
    );

    samples
}
