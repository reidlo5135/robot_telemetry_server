use crate::db::writer::SqliteWriterHandle;
use rclrs::Node;

pub struct TelemetryState {
    pub node: Node,
    pub db_writer: SqliteWriterHandle,
}
