use anyhow::{Context, Result};
use rusqlite::{Connection, params};
use std::{
    path::{Path, PathBuf},
    sync::mpsc::{self, Receiver, Sender},
    thread,
    time::Duration,
};

use super::schema;

#[derive(Clone, Debug)]
pub(crate) struct MetricSample {
    pub(crate) timestamp_ns: i64,
    pub(crate) source_topic: String,
    pub(crate) metric_name: String,
    pub(crate) metric_value: f64,
    pub(crate) unit: String,
    pub(crate) quality: String,
}

#[derive(Clone, Debug)]
pub(crate) struct EventLog {
    pub(crate) timestamp_ns: i64,
    pub(crate) source_topic: String,
    pub(crate) logger_name: String,
    pub(crate) severity: i64,
    pub(crate) message: String,
    pub(crate) file: Option<String>,
    pub(crate) function_name: Option<String>,
    pub(crate) line: Option<i64>,
}

#[derive(Clone)]
pub(crate) struct SqliteWriterHandle {
    sender: Sender<WriteCommand>,
}

enum WriteCommand {
    Metric(MetricSample),
    Event(EventLog),
}

impl SqliteWriterHandle {
    pub(crate) fn spawn(path: impl Into<PathBuf>) -> Result<Self> {
        let path = path.into();
        create_parent_dir(&path)?;

        let connection = Connection::open(&path)
            .with_context(|| format!("failed to open sqlite database at {}", path.display()))?;
        configure_connection(&connection)?;
        schema::initialize(&connection).context("failed to initialize sqlite schema")?;

        let (sender, receiver) = mpsc::channel();
        thread::Builder::new()
            .name("robot_telemetry_sqlite_writer".to_string())
            .spawn(move || run_writer(connection, receiver))
            .context("failed to spawn sqlite writer thread")?;

        Ok(Self { sender })
    }

    pub(crate) fn write_metric(&self, sample: MetricSample) -> Result<()> {
        self.sender
            .send(WriteCommand::Metric(sample))
            .context("failed to queue metric sample for sqlite writer")
    }

    pub(crate) fn write_event(&self, event: EventLog) -> Result<()> {
        self.sender
            .send(WriteCommand::Event(event))
            .context("failed to queue event log for sqlite writer")
    }
}

fn create_parent_dir(path: &Path) -> Result<()> {
    let Some(parent) = path.parent() else {
        return Ok(());
    };

    if parent.as_os_str().is_empty() {
        return Ok(());
    }

    std::fs::create_dir_all(parent)
        .with_context(|| format!("failed to create sqlite directory {}", parent.display()))
}

fn configure_connection(connection: &Connection) -> rusqlite::Result<()> {
    connection.busy_timeout(Duration::from_secs(5))?;
    connection.execute_batch(
        r#"
		PRAGMA journal_mode = WAL;
		PRAGMA synchronous = NORMAL;
		PRAGMA foreign_keys = ON;
		"#,
    )?;
    Ok(())
}

fn run_writer(mut connection: Connection, receiver: Receiver<WriteCommand>) {
    while let Ok(command) = receiver.recv() {
        let result = match command {
            WriteCommand::Metric(sample) => insert_metric(&mut connection, &sample),
            WriteCommand::Event(event) => insert_event(&mut connection, &event),
        };

        if let Err(error) = result {
            eprintln!("sqlite writer failed: {error}");
        }
    }
}

fn insert_metric(connection: &mut Connection, sample: &MetricSample) -> rusqlite::Result<()> {
    connection.execute(
        schema::INSERT_METRIC_SAMPLE,
        params![
            sample.timestamp_ns,
            sample.source_topic,
            sample.metric_name,
            sample.metric_value,
            sample.unit,
            sample.quality,
        ],
    )?;
    Ok(())
}

fn insert_event(connection: &mut Connection, event: &EventLog) -> rusqlite::Result<()> {
    connection.execute(
        schema::INSERT_EVENT_LOG,
        params![
            event.timestamp_ns,
            event.source_topic,
            event.logger_name,
            event.severity,
            event.message,
            event.file,
            event.function_name,
            event.line,
        ],
    )?;
    Ok(())
}
