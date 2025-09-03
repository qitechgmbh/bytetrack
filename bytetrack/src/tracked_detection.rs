use crate::{BBox, Detection};

#[derive(Clone)]
pub struct TrackedDetection {
    // Detection in the frame
    pub detection: Option<Detection>,
    // The index of the detection in the detections array passed to the [`Bytetrack::track`] method
    pub detection_index: Option<usize>,
    // Kalman predicted bounding box in the frame
    pub kalman_bbox: BBox,
    // Frame index
    pub frame_index: usize,
}

impl TrackedDetection {
    pub fn new(
        detection: Option<&Detection>,
        detection_index: Option<usize>,
        kalman_bbox: BBox,
        frame_index: usize,
    ) -> Self {
        Self {
            detection: detection.map(|x| x.clone()),
            detection_index,
            kalman_bbox,
            frame_index,
        }
    }
}
