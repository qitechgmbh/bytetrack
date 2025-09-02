# ByteTrack: Multi-Object Tracking Library

This library provides a Rust port of the [Python library](https://github.com/FoundationVision/ByteTrack) of the [ByteTrack algorithm](https://arxiv.org/abs/2110.06864) for multi-object tracking (MOT) in real-time and computer vision applications. ByteTrack is also in [Multi-Object Tracking with Ultralytics YOLO](https://docs.ultralytics.com/de/modes/track/).

## Features

- Idiomatic API
- Access historical
  - tracked positions
  - detected bounding boxes (your input)
  - predicted bounding boxes (Kalman filter)
  - tracked/lost state
- Configure ByteTrack & Kalman filter parameters
- Customize ID generation (e.g. random UUIDs, incremental u32, etc.)

## Example

```rust
use bytetrack::{Detection, Bytetrack, BytetrackConfig};

fn main() {
    let mut total_tracked_objects = 0;

    let frames: Vec<Vec<Detection>> = // insert frames with detection here

    // Initializing tracker which uses u32 unique incremental IDs for tracked objects
    let mut tracker = Bytetrack::new(BytetrackConfig::<u32>::default());

    // Process each frame
    for (i, frame) in frames.iter().enumerate() {
        println!("Processing frame {} with {} detections", i, frame.len());

        // Track objects in the current frame
        let result = tracker.track(&frame_detections);

        // Count removed objects
        let removed_count = result.removed_objects.len();
        total_tracked_objects += result.new_objects.len();

        // Read tracking results or access tracker.get_objects() directly
        println!("New object detected: {:?}", result.new_objects.len());
        println!("Updated objects: {:?}", result.updated_objects.len());
        println!("Removed objects: {:?}", removed_count);
    }

    // Read final tracked objects and clear internal state
    let final_count = tracker.drain_objects();
    total_tracked_objects += final_count.len();
    println!("Total tracked objects: {:?}", total_tracked_objects);

    // Print track of some object
    if let Some(object) = final_objects.last() {
        println!(" Track of object {}:", object.id);
        for t in object.track.iter() {
            println!(
                "Frame: {}, X: {}, Y: {}",
                t.frame_index,
                t.bbox.center().x,
                t.bbox.center().y
            )
        }
    }
}
```