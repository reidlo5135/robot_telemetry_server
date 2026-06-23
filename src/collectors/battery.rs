use sensor_msgs::msg::BatteryState;

use crate::collectors::common::{
    finite_stats, push_count_sample, push_finite_sample, timestamp_ns,
};
use crate::db::writer::MetricSample;

pub(crate) fn extract_battery_metrics(msg: &BatteryState, source_topic: &str) -> Vec<MetricSample> {
    let timestamp_ns = timestamp_ns(msg.header.stamp.sec, msg.header.stamp.nanosec);
    let mut samples = Vec::with_capacity(19);

    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_voltage",
        f64::from(msg.voltage),
        "V",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_current",
        f64::from(msg.current),
        "A",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_temperature",
        f64::from(msg.temperature),
        "C",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_charge",
        f64::from(msg.charge),
        "Ah",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_capacity",
        f64::from(msg.capacity),
        "Ah",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_design_capacity",
        f64::from(msg.design_capacity),
        "Ah",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_percentage",
        f64::from(msg.percentage),
        "ratio",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_power_supply_status",
        f64::from(msg.power_supply_status),
        "code",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_power_supply_health",
        f64::from(msg.power_supply_health),
        "code",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_power_supply_technology",
        f64::from(msg.power_supply_technology),
        "code",
    );
    push_finite_sample(
        &mut samples,
        timestamp_ns,
        source_topic,
        "battery_present",
        f64::from(u8::from(msg.present)),
        "bool",
    );

    if let Some(cell_voltage_stats) = finite_stats(msg.cell_voltage.iter().copied().map(f64::from))
    {
        push_count_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "battery_cell_voltage_count",
            cell_voltage_stats.count,
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "battery_cell_voltage_min",
            cell_voltage_stats.min,
            "V",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "battery_cell_voltage_max",
            cell_voltage_stats.max,
            "V",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "battery_cell_voltage_mean",
            cell_voltage_stats.mean,
            "V",
        );
    }

    if let Some(cell_temperature_stats) =
        finite_stats(msg.cell_temperature.iter().copied().map(f64::from))
    {
        push_count_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "battery_cell_temperature_count",
            cell_temperature_stats.count,
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "battery_cell_temperature_min",
            cell_temperature_stats.min,
            "C",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "battery_cell_temperature_max",
            cell_temperature_stats.max,
            "C",
        );
        push_finite_sample(
            &mut samples,
            timestamp_ns,
            source_topic,
            "battery_cell_temperature_mean",
            cell_temperature_stats.mean,
            "C",
        );
    }

    samples
}
