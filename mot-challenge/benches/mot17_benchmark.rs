//! ByteTrack Performance Benchmark
//!
//! This benchmark measures the performance of ByteTrack processing
//! the first 10 frames of a MOT17 sequence.

use anyhow::Result;
use bytetrack::{Bytetrack, BytetrackConfig};
use criterion::{Criterion, black_box, criterion_group, criterion_main};
use mot_challenge::mot17::{DatasetType, DetectorType, MotDataset, SequenceConfig};
use std::{default, path::Path};

/// Load the first 10 frames of a MOT17 sequence for benchmarking
fn load_benchmark_data(n: usize) -> Result<Vec<Vec<bytetrack::Detection>>> {
    let dataset_root = Path::new("../datasets/MOT17");
    let dataset = MotDataset::new(dataset_root);

    let config = SequenceConfig::new("MOT17-02", DetectorType::FRCNN, DatasetType::TrainDetection);
    let sequence = dataset.load_sequence(&config)?;

    // Take only the first 10 frames
    let frames_to_process = sequence.frames.into_iter().take(n);

    let mut benchmark_frames = Vec::new();

    for frame in frames_to_process {
        // Convert detections to ByteTrack format
        let detections: Vec<bytetrack::Detection> =
            frame.detections.into_iter().map(|det| det.into()).collect();

        benchmark_frames.push(detections);
    }

    Ok(benchmark_frames)
}

/// Benchmark function that processes 10 frames through ByteTrack
fn benchmark_hungarian(frames: &[Vec<bytetrack::Detection>]) -> Result<()> {
    // Initialize ByteTrack with default configuration
    let mut tracker = Bytetrack::new(BytetrackConfig::<u32> {
        max_disappeared: 30,
        ..default::Default::default()
    });

    // Process each frame
    for detections in frames {
        // Create references for tracking (ByteTrack expects &[&Detection])
        let detection_refs: Vec<&bytetrack::Detection> = detections.iter().collect();

        // Run tracking on this frame - this is what we're benchmarking
        let _result = tracker.track(black_box(&detection_refs));
    }

    Ok(())
}

fn criterion_benchmark(c: &mut Criterion) {
    // Load the benchmark data once
    let frames = load_benchmark_data(10)
        .expect("Failed to load benchmark data. Make sure the MOT17 dataset is available.");

    println!("Loaded {} frames for benchmarking", frames.len());

    // Benchmark per-frame performance by dividing the result
    let mut group = c.benchmark_group("bytetrack");
    group.bench_function("hungarian_frame", |b| {
        b.iter_custom(|iters| {
            let start = std::time::Instant::now();
            for _ in 0..iters {
                benchmark_hungarian(black_box(&frames)).expect("Benchmark tracking failed");
            }
            let elapsed = start.elapsed();
            // Divide by number of frames to get per-frame timing
            elapsed / frames.len() as u32
        })
    });
    group.finish();
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
