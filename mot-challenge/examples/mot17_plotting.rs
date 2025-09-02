//! Basic MOT17 Dataset Loading and ByteTrack Example
//!
//! This example demonstrates how to:
//! 1. Load a MOT17 dataset sequence
//! 2. Initialize a ByteTrack tracker
//! 3. Process detections frame by frame
//! 4. Track objects across frames
//! 5. Generate a plot visualization of tracked objects

use anyhow::Result;
use bytetrack::{Bytetrack, BytetrackConfig};
use mot_challenge::mot17::{DatasetType, DetectorType, MotDataset, SequenceConfig};
use plotters::prelude::*;
use std::{collections::HashMap, default, path::Path};

fn main() -> Result<()> {
    // Initialize the dataset
    let dataset_root = Path::new("../datasets/MOT17");
    let dataset = MotDataset::new(dataset_root);

    // Configure which sequence to load
    let config = SequenceConfig::new("MOT17-02", DetectorType::FRCNN, DatasetType::TrainDetection);

    println!("Loading sequence: {}", config.sequence_name);

    // Load the sequence
    let sequence = dataset.load_sequence(&config)?;

    println!("Loaded sequence with {} frames", sequence.frames.len());

    // Initialize ByteTrack with default configuration
    let mut tracker = Bytetrack::new(BytetrackConfig::<u32> {
        max_disappeared: 10,
        ..default::Default::default()
    });

    let mut objects = Vec::new();
    let mut track_trajectories: HashMap<u32, Vec<(f32, f32, usize)>> = HashMap::new(); // object_id -> [(x, y, frame_id)]

    // Process each frame
    let mut last_i = 0;
    for (i, frame) in sequence.frames.iter().take(100).enumerate() {
        // Convert detections to ByteTrack format
        let detections: Vec<bytetrack::Detection> = frame
            .detections
            .iter()
            .map(|det| det.clone().into())
            .collect();

        // Create references for tracking (ByteTrack expects &[&Detection])
        let detection_refs: Vec<&bytetrack::Detection> = detections.iter().collect();

        // Run tracking on this frame
        let result = tracker.track(&detection_refs);

        // Store trajectory points for all active objects
        for (object_id, object) in tracker.objects() {
            let bbox = object.current_bbox();
            let center_x = bbox.left() + bbox.width() / 2.0;
            let center_y = bbox.top() + bbox.height() / 2.0;

            track_trajectories.entry(*object_id).or_default().push((
                center_x,
                center_y,
                frame.frame_id as usize,
            ));
        }

        // Save removed objects for analysis
        objects.extend(result.removed_objects);

        if (i + 1) % 10 == 0 {
            println!("Processed {} frames", i + 1);
        }
        last_i = i;
    }

    println!("Processed a total of {} frames", last_i + 1);

    // Extract final tracks
    objects.extend(tracker.drain_objects());

    // Generate plot of tracked objects
    create_tracking_plot(&track_trajectories)?;

    println!("Tracking visualization saved to 'tracking_plot.png'");

    Ok(())
}

/// Create a plot visualization of all tracked object trajectories
fn create_tracking_plot(track_trajectories: &HashMap<u32, Vec<(f32, f32, usize)>>) -> Result<()> {
    let root = BitMapBackend::new("mot17_plotting.png", (1280, 720)).into_drawing_area();
    root.fill(&WHITE)?;

    // Find the bounds of all trajectories
    let mut min_x = f32::INFINITY;
    let mut max_x = f32::NEG_INFINITY;
    let mut min_y = f32::INFINITY;
    let mut max_y = f32::NEG_INFINITY;

    for trajectory in track_trajectories.values() {
        for (x, y, _) in trajectory {
            min_x = min_x.min(*x);
            max_x = max_x.max(*x);
            min_y = min_y.min(*y);
            max_y = max_y.max(*y);
        }
    }

    // Add some padding
    let padding = 50.0;
    min_x -= padding;
    max_x += padding;
    min_y -= padding;
    max_y += padding;

    let mut chart = ChartBuilder::on(&root)
        .caption("ByteTrack Object Trajectories", ("Arial", 30))
        .margin(20)
        .x_label_area_size(40)
        .y_label_area_size(50)
        .build_cartesian_2d(min_x..max_x, min_y..max_y)?;

    chart
        .configure_mesh()
        .x_desc("X Position (pixels)")
        .y_desc("Y Position (pixels)")
        .draw()?;

    // Generate different colors for each track
    let colors = [
        &RED,
        &BLUE,
        &GREEN,
        &MAGENTA,
        &CYAN,
        &BLACK,
        &RGBColor(255, 165, 0),   // Orange
        &RGBColor(128, 0, 128),   // Purple
        &RGBColor(255, 192, 203), // Pink
        &RGBColor(165, 42, 42),   // Brown
    ];

    // Draw trajectories for each object
    for (i, (object_id, trajectory)) in track_trajectories.iter().enumerate() {
        if trajectory.len() < 2 {
            continue; // Skip objects with less than 2 points
        }

        let color = colors[i % colors.len()];

        // Draw trajectory line
        let points: Vec<(f32, f32)> = trajectory.iter().map(|(x, y, _)| (*x, *y)).collect();
        chart
            .draw_series(LineSeries::new(points.iter().cloned(), color))?
            .label(format!("Object {}", object_id))
            .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 10, y)], color));

        // Draw start point (green circle)
        if let Some((start_x, start_y, _)) = trajectory.first() {
            chart.draw_series(std::iter::once(Circle::new(
                (*start_x, *start_y),
                4,
                GREEN.filled(),
            )))?;
        }

        // Draw end point (red circle)
        if let Some((end_x, end_y, _)) = trajectory.last() {
            chart.draw_series(std::iter::once(Circle::new(
                (*end_x, *end_y),
                4,
                RED.filled(),
            )))?;
        }
    }

    chart
        .configure_series_labels()
        .background_style(&WHITE.mix(0.8))
        .border_style(&BLACK)
        .draw()?;

    root.present()?;
    Ok(())
}
