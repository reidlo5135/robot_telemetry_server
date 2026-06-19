
pub struct MetricsSample {
    pub timestamp_ns: i64,
    pub source_topic: String,
    pub metric_name: String,
    pub metric_value: f64,
    pub unit: String,
    pub quality: String,
}