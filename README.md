robot_telemetry_server/
├── Cargo.toml
├── package.xml
├── README.md
├── config/
│   └── telemetry.yaml
├── launch/
│   └── robot_telemetry_server.launch.py
└── src/
├── main.rs
├── telemetry_node.rs
├── state.rs
├── metric.rs
├── db/
│   ├── mod.rs
│   ├── schema.rs
│   └── sqlite_writer.rs
├── collectors/
│   ├── mod.rs
│   ├── odom.rs
│   ├── battery.rs
│   ├── cmd_vel.rs
│   └── rosout.rs
└── util/
├── mod.rs
└── time.rs


main.rs
- Context 생성
- Executor 생성
- TelemetryNode 생성
- executor.spin()

telemetry_node.rs
- rclrs Node 소유
- Worker 생성
- subscription 생성/보관
- 기존 rclcpp Node 클래스 역할

state.rs
- Worker callback에서 공유할 상태
- DB writer handle
- message counter
- config 보관

metric.rs
- metric_samples에 넣을 MetricSample struct
- event_logs에 넣을 EventLog struct
- metric_name/unit/quality 정의

db/schema.rs
- CREATE TABLE SQL
- index 생성 SQL

db/sqlite_writer.rs
- SQLite connection
- insert_metric()
- insert_event()
- busy_timeout / WAL 설정

collectors/odom.rs
- nav_msgs/Odometry → odom_linear_x, odom_angular_z 변환

collectors/battery.rs
- sensor_msgs/BatteryState → battery_voltage, battery_percentage 변환

collectors/cmd_vel.rs
- geometry_msgs/Twist → cmd_vel_linear_x, cmd_vel_angular_z 변환

collectors/rosout.rs
- rcl_interfaces/Log → event_logs 변환

util/time.rs
- ROS time / system time → timestamp_ns 변환