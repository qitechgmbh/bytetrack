use nalgebra::Point2;

use crate::{
    bbox::BBox,
    detection::Detection,
    kalman::{BBoxKalmanConfig, BBoxKalmanFilter},
};

#[derive(Clone, Debug)]
pub enum ObjectStatus {
    Tracked,
    Lost { frames: u32 },
}

impl ObjectStatus {
    pub fn incrment_lost_frames(&mut self) {
        match self {
            ObjectStatus::Tracked => {
                *self = ObjectStatus::Lost { frames: 1 };
            }
            ObjectStatus::Lost { frames } => {
                *frames += 1;
            }
        }
    }

    pub fn get_lost_frames(&self) -> u32 {
        match self {
            ObjectStatus::Tracked => 0,
            ObjectStatus::Lost { frames } => *frames,
        }
    }

    pub fn reset(&mut self) {
        *self = ObjectStatus::Tracked;
    }
}

pub struct Object {
    /// Detection that was last given to the tracker via [`Bytetrack::track`]
    pub detection: Detection,
    /// Index of the last detection when given to the [`Bytetrack::track`] method
    pub detection_index: Option<usize>,
    /// Bounding box updated by Kalman filter
    pub bbox: BBox,
    /// Tracked points
    pub points: Vec<Point2<f32>>,
    /// Status of the track
    pub status: ObjectStatus,
    /// lost frames
    /// Kalman filter for predicting bbox position and size
    kalman_filter: BBoxKalmanFilter,
}

impl Clone for Object {
    fn clone(&self) -> Self {
        // We'll recreate the Kalman filter when cloning
        let mut cloned = Object::from_detection(self.detection.clone());
        cloned.detection_index = self.detection_index;
        cloned.bbox = self.bbox.clone();
        cloned.points = self.points.clone();
        cloned.status = self.status.clone();
        cloned
    }
}

impl Object {
    pub fn from_detection(detection: Detection) -> Self {
        // Create Kalman filter with default configuration
        let config = BBoxKalmanConfig::default();
        let kalman_filter = BBoxKalmanFilter::new(&detection.bbox, config).unwrap();

        Self {
            bbox: detection.bbox.clone(),
            detection,
            detection_index: None,
            points: Vec::new(),
            status: ObjectStatus::Tracked,
            kalman_filter,
        }
    }

    pub fn new_with_config(detection: Detection, config: BBoxKalmanConfig) -> Self {
        // Create Kalman filter with custom configuration
        let kalman_filter = BBoxKalmanFilter::new(&detection.bbox, config).unwrap();

        Self {
            bbox: detection.bbox.clone(),
            detection,
            detection_index: None,
            points: Vec::new(),
            status: ObjectStatus::Tracked,
            kalman_filter,
        }
    }

    pub fn predict(&mut self) -> BBox {
        // Perform Kalman filter prediction and get predicted bbox
        let predicted_bbox = self.kalman_filter.predict();
        self.bbox = predicted_bbox.clone();
        predicted_bbox
    }

    pub fn update(&mut self, detection: Detection) -> Result<(), Box<dyn std::error::Error>> {
        // Update Kalman filter with new detection
        self.kalman_filter.update(&detection.bbox)?;

        // Update stored detection and bbox from filter state
        self.detection = detection;
        self.bbox = self.kalman_filter.current_bbox();

        // update status
        self.status = ObjectStatus::Tracked;

        Ok(())
    }
}
