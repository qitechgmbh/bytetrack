use crate::{BBox, Detection};

#[derive(Clone)]
pub struct TrackedDetection {
    // Detection in the frame
    pub detection: Detection,
    // The index of the detection in the detections array passed to the [`Bytetrack::track`] method
    pub detection_index: usize,
    // Kalman predicted bounding box in the frame
    pub bbox: BBox,
    // Frame index
    pub frame_index: usize,
}

impl TrackedDetection {
    pub fn new(detection: &Detection, detection_index: usize, frame_index: usize) -> Self {
        Self {
            detection: detection.clone(),
            detection_index,
            bbox: detection.bbox.clone(),
            frame_index,
        }
    }
}
