use rusqlite::Connection;

pub(crate) const CREATE_TABLES: &str = r#"
CREATE TABLE IF NOT EXISTS metric_samples (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	timestamp_ns INTEGER NOT NULL,
	source_topic TEXT NOT NULL,
	metric_name TEXT NOT NULL,
	metric_value REAL NOT NULL,
	unit TEXT NOT NULL,
	quality TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS event_logs (
	id INTEGER PRIMARY KEY AUTOINCREMENT,
	timestamp_ns INTEGER NOT NULL,
	source_topic TEXT NOT NULL,
	logger_name TEXT NOT NULL,
	severity INTEGER NOT NULL,
	message TEXT NOT NULL,
	file TEXT,
	function_name TEXT,
	line INTEGER
);
"#;

pub(crate) const CREATE_INDEXES: &str = r#"
CREATE INDEX IF NOT EXISTS idx_metric_samples_time
ON metric_samples(timestamp_ns);

CREATE INDEX IF NOT EXISTS idx_metric_samples_name_time
ON metric_samples(metric_name, timestamp_ns);

CREATE INDEX IF NOT EXISTS idx_event_logs_time
ON event_logs(timestamp_ns);

CREATE INDEX IF NOT EXISTS idx_event_logs_severity_time
ON event_logs(severity, timestamp_ns);
"#;

pub(crate) const INSERT_METRIC_SAMPLE: &str = r#"
INSERT INTO metric_samples (
	timestamp_ns,
	source_topic,
	metric_name,
	metric_value,
	unit,
	quality
) VALUES (?1, ?2, ?3, ?4, ?5, ?6)
"#;

pub(crate) const INSERT_EVENT_LOG: &str = r#"
INSERT INTO event_logs (
	timestamp_ns,
	source_topic,
	logger_name,
	severity,
	message,
	file,
	function_name,
	line
) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8)
"#;

pub(crate) fn initialize(connection: &Connection) -> rusqlite::Result<()> {
    connection.execute_batch(CREATE_TABLES)?;
    connection.execute_batch(CREATE_INDEXES)?;
    Ok(())
}
