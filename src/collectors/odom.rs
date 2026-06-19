use nav_msgs::msg::Odometry;

pub fn extract_odom_metric(msg: &Odometry) -> (f64, f64) {
    let linear_x: f64 = msg.twist.twist.linear.x;
    let angular_z: f64 = msg.twist.twist.angular.z;

    (linear_x, angular_z)
}
