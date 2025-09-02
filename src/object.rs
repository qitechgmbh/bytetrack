use nalgebra::Point2;

use crate::{
    bbox::BBox,
    detection::Detection,
    kalman::{BBoxKalmanConfig, BBoxKalmanFilter},
};

/// Status of a tracked object
///
/// Objects can be either actively tracked (when recently matched to detections)
/// or temporarily lost (when no matching detection was found for some frames).
#[derive(Clone, Debug)]
pub enum ObjectStatus {
    /// Object is actively being tracked
    Tracked,
    /// Object is temporarily lost for the specified number of frames
    Lost { frames: u32 },
}

impl ObjectStatus {
    /// Increment the number of lost frames or transition from Tracked to Lost
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

    /// Get the number of frames this object has been lost
    ///
    /// Returns 0 if the object is currently tracked.
    pub fn get_lost_frames(&self) -> u32 {
        match self {
            ObjectStatus::Tracked => 0,
            ObjectStatus::Lost { frames } => *frames,
        }
    }

    /// Set status to Tracked
    pub fn set_tracked(&mut self) {
        *self = ObjectStatus::Tracked;
    }
}

/// A tracked object with Kalman filter-based prediction
///
/// This represents an object being tracked across multiple frames. It maintains
/// the latest detection information, predicted bounding box from Kalman filtering,
/// tracking status, and optional point features.
pub struct Object {
    /// Detection that was last given to the tracker via `Bytetrack::track`
    pub detection: Detection,
    /// Index of the last detection when given to the `Bytetrack::track` method
    pub detection_index: Option<usize>,
    /// Bounding box updated by Kalman filter
    pub bbox: BBox,
    /// Tracked points (for future feature tracking extensions)
    pub points: Vec<Point2<f32>>,
    /// Status of the track (Tracked or Lost with frame count)
    pub status: ObjectStatus,
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
    /// Create a new Object from a detection with default Kalman filter configuration
    ///
    /// # Arguments
    /// * `detection` - Initial detection to create the object from
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

    /// Create a new Object with custom Kalman filter configuration
    ///
    /// # Arguments
    /// * `detection` - Initial detection to create the object from
    /// * `config` - Custom Kalman filter configuration
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

    /// Predict the next bounding box position using Kalman filter
    ///
    /// This should be called once per frame to get the predicted location
    /// of the object before attempting to match it with new detections.
    ///
    /// # Returns
    /// Predicted bounding box for the next frame
    pub fn predict(&mut self) -> BBox {
        // Perform Kalman filter prediction and get predicted bbox
        let predicted_bbox = self.kalman_filter.predict();
        self.bbox = predicted_bbox.clone();
        predicted_bbox
    }

    /// Update the object with a new matching detection
    ///
    /// This updates the Kalman filter with the new detection and resets
    /// the object status to Tracked.
    ///
    /// # Arguments
    /// * `detection` - New detection that matches this object
    ///
    /// # Returns
    /// Result indicating success or Kalman filter error
    pub fn update(&mut self, detection: Detection) -> Result<(), Box<dyn std::error::Error>> {
        // Update Kalman filter with new detection
        self.kalman_filter.update(&detection.bbox)?;

        // Update stored detection and bbox from filter state
        self.detection = detection;
        self.bbox = self.kalman_filter.current_bbox();

        // update status
        self.status.set_tracked();

        Ok(())
    }
}
