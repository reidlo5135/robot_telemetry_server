# robot_telemetry_server

Rust `rclrs` 기반 ROS 2 telemetry 수집 노드입니다. 여러 ROS topic을 subscribe하고, 일부 telemetry 값을 SQLite database에 기록합니다.

현재 DB write 연결은 `/odom` metric과 `/rosout` event log부터 적용되어 있습니다.

## Project Structure

```text
robot_telemetry_server/
├── Cargo.toml
├── package.xml
├── README.md
├── config/
│   └── robot_telemetry_server.yaml
├── launch/
│   └── robot_telemetry_server.launch.xml
└── src/
    ├── main.rs
    ├── lib.rs
    ├── state.rs
    ├── rcl/
    │   ├── mod.rs
    │   ├── node.rs
    │   └── params.rs
    ├── db/
    │   ├── mod.rs
    │   ├── schema.rs
    │   └── writer.rs
    └── collectors/
        ├── mod.rs
        ├── odom.rs
        ├── battery.rs
        ├── rosout.rs
        ├── tf.rs
        └── twist.rs
```

## Module Roles

`src/main.rs`
- ROS context 생성
- executor 생성
- `RobotTelemetryServerNode` 생성
- `executor.spin()` 실행

`src/lib.rs`
- crate 내부 모듈 선언
- `RobotTelemetryServerNode`를 crate root로 re-export

`src/rcl/node.rs`
- `rclrs` node 생성
- worker 생성
- subscription 생성 및 보관
- callback에서 collector와 DB writer 호출
- C++ `rclcpp::Node` 클래스에 가까운 역할

`src/rcl/params.rs`
- ROS parameter declare/get 처리
- topic별 QoS 설정 변환
- DB path, batch size, flush interval parameter 처리
- parameter handle 보관 구조 정의

`src/state.rs`
- worker callback에서 공유하는 상태
- `Node` logger handle 보관
- SQLite writer handle 보관

`src/db/schema.rs`
- SQLite table 생성 SQL
- index 생성 SQL
- insert SQL 상수

`src/db/writer.rs`
- `rusqlite` connection 관리
- WAL, busy timeout 설정
- background writer thread 생성
- metric/event write command 처리

`src/collectors/odom.rs`
- `nav_msgs/Odometry`에서 `linear.x`, `angular.z` 추출

`src/collectors/*.rs`
- topic별 telemetry 변환 로직을 둘 자리
- `battery`, `twist`, `tf`, `rosout` collector는 확장 예정

## Parameters

기본 parameter file은 `config/robot_telemetry_server.yaml`입니다.

주요 설정:

```yaml
robot_telemetry_server:
  ros__parameters:
    db:
      path: "/home/rnc/ws/data/db/telemetry.db"
      batch_size: 100
      flush_interval_ms: 1000
    odom:
      topic: "/odom"
      qos:
        reliability: "reliable"
        depth: 10
        durability: "volatile"
```

Rust 쪽에서는 nested YAML key를 dot path로 declare합니다.

```text
db.path
db.batch_size
odom.topic
odom.qos.reliability
odom.qos.depth
odom.qos.durability
```

## SQLite Output

현재 DB에는 두 테이블을 생성합니다.

```text
metric_samples
event_logs
```

현재 기록 대상:

```text
/odom   -> metric_samples: odom_linear_x, odom_angular_z
/rosout -> event_logs
```

DB 파일 확인:

```bash
sqlite3 /home/rnc/ws/data/db/telemetry.db ".tables"
sqlite3 /home/rnc/ws/data/db/telemetry.db ".schema"
```

metric 확인:

```bash
sqlite3 /home/rnc/ws/data/db/telemetry.db \
  -cmd ".headers on" \
  -cmd ".mode column" \
  "SELECT timestamp_ns, source_topic, metric_name, metric_value, unit, quality FROM metric_samples LIMIT 20;"
```

event log 확인:

```bash
sqlite3 /home/rnc/ws/data/db/telemetry.db \
  -cmd ".headers on" \
  -cmd ".mode column" \
  "SELECT timestamp_ns, logger_name, severity, message FROM event_logs LIMIT 20;"
```

WAL mode를 사용하므로 실행 중에는 다음 파일들이 함께 생길 수 있습니다.

```text
telemetry.db
telemetry.db-wal
telemetry.db-shm
```

## Build And Check

```bash
cargo fmt
cargo check
```

ROS workspace에서는 일반적으로 `colcon`/`ament_cargo` 흐름으로 빌드합니다.

```bash
colcon build --packages-select robot_telemetry_server
```

## Launch

```bash
ros2 launch robot_telemetry_server robot_telemetry_server.launch.xml
```

launch file은 `config/robot_telemetry_server.yaml`을 parameter file로 로드합니다.
