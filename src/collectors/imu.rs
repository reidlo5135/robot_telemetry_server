use sensor_msgs::msg::Imu;

use crate::collectors::common::{push_finite_sample, timestamp_ns};
use crate::db::writer::MetricSample;

pub(crate) fn extract_imu_metrics(msg: &Imu, source_topic: &str) -> Vec<MetricSample> {
    let timestamp_ns = timestamp_ns(msg.header.stamp.sec, msg.header.stamp.nanosec);
    let mut samples = Vec::with_capacity(10);

    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "imu_orientation_x",
        msg.orientation.x,
        "quaternion",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "imu_orientation_y",
        msg.orientation.y,
        "quaternion",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "imu_orientation_z",
        msg.orientation.z,
        "quaternion",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "imu_orientation_w",
        msg.orientation.w,
        "quaternion",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "imu_angular_velocity_x",
        msg.angular_velocity.x,
        "rad/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "imu_angular_velocity_y",
        msg.angular_velocity.y,
        "rad/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "imu_angular_velocity_z",
        msg.angular_velocity.z,
        "rad/s",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "imu_linear_acceleration_x",
        msg.linear_acceleration.x,
        "m/s^2",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "imu_linear_acceleration_y",
        msg.linear_acceleration.y,
        "m/s^2",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "imu_linear_acceleration_z",
        msg.linear_acceleration.z,
        "m/s^2",
    );

    samples
}
