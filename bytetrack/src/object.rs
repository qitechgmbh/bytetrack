use std::hash::Hash;

use crate::{
    ObjectStatus, TrackedDetection,
    bbox::BBox,
    detection::Detection,
    kalman::{BBoxKalmanConfig, BBoxKalmanFilter},
};

/// A tracked object with Kalman filter-based prediction
///
/// This represents an object being tracked across multiple frames. It maintains
/// the latest detection information, predicted bounding box from Kalman filtering,
/// tracking status, and optional point features.
pub struct Object<ID>
where
    ID: Hash + Eq + Clone,
{
    pub id: ID,
    /// Tracked points (for future feature tracking extensions)
    ///
    /// Always contains at least one entry (the initial detection)
    pub track: Vec<TrackedDetection>,
    /// Status of the track (Tracked or Lost with frame count)
    pub status: ObjectStatus,
    /// Kalman filter for predicting bbox position and size
    kalman_filter: BBoxKalmanFilter,
}

impl<ID> Clone for Object<ID>
where
    ID: Hash + Eq + Clone,
{
    fn clone(&self) -> Self {
        // Recreate the Kalman filter since it doesn't implement Clone
        let kalman_filter = if let Some(latest_detection) = self.track.last() {
            let config = BBoxKalmanConfig::default();
            BBoxKalmanFilter::new(&latest_detection.detection.bbox, config).unwrap()
        } else {
            unreachable!("Object must have at least one tracked detection to clone")
        };

        Self {
            id: self.id.clone(),
            track: self.track.clone(),
            status: self.status.clone(),
            kalman_filter,
        }
    }
}

impl<ID> Object<ID>
where
    ID: Hash + Eq + Clone,
{
    /// Create a new Object from a detection with default Kalman filter configuration
    ///
    /// # Arguments
    /// * `id` - Unique identifier for this object
    /// * `detection` - Initial detection to create the object from
    /// * `detection_index` - Index of the detection in the detections array
    /// * `frame_index` - Current frame index
    pub fn from_detection(
        id: ID,
        detection: &Detection,
        detection_index: usize,
        frame_index: usize,
    ) -> Self {
        // Create Kalman filter with default configuration
        let config = BBoxKalmanConfig::default();
        let kalman_filter = BBoxKalmanFilter::new(&detection.bbox, config).unwrap();

        let tracked_detection = TrackedDetection::new(detection, detection_index, frame_index);

        Self {
            id,
            track: vec![tracked_detection],
            status: ObjectStatus::Tracked,
            kalman_filter,
        }
    }

    /// Create a new Object with custom Kalman filter configuration
    ///
    /// # Arguments
    /// * `id` - Unique identifier for this object
    /// * `detection` - Initial detection to create the object from
    /// * `detection_index` - Index of the detection in the detections array
    /// * `frame_index` - Current frame index
    /// * `config` - Custom Kalman filter configuration
    pub fn new_with_config(
        id: ID,
        detection: &Detection,
        detection_index: usize,
        frame_index: usize,
        config: BBoxKalmanConfig,
    ) -> Self {
        // Create Kalman filter with custom configuration
        let kalman_filter = BBoxKalmanFilter::new(&detection.bbox, config).unwrap();

        let tracked_detection = TrackedDetection::new(detection, detection_index, frame_index);

        Self {
            id,
            track: vec![tracked_detection],
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

        // Update the latest tracked detection's bbox with the prediction
        if let Some(last_detection) = self.track.last_mut() {
            last_detection.bbox = predicted_bbox.clone();
        }

        predicted_bbox
    }

    /// Update the object with a new matching detection
    ///
    /// This updates the Kalman filter with the new detection and adds
    /// a new tracked detection to the track history. Also resets
    /// the object status to Tracked.
    ///
    /// # Arguments
    /// * `detection` - New detection that matches this object
    /// * `detection_index` - Index of the detection in the detections array
    /// * `frame_index` - Current frame index
    ///
    /// # Returns
    /// Result indicating success or Kalman filter error
    pub fn update(
        &mut self,
        detection: &Detection,
        detection_index: usize,
        frame_index: usize,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Update Kalman filter with new detection
        self.kalman_filter.update(&detection.bbox)?;

        // Add new tracked detection to the track history
        let tracked_detection = TrackedDetection::new(detection, detection_index, frame_index);
        self.track.push(tracked_detection);

        // Update status to tracked
        self.status.set_tracked();

        Ok(())
    }

    /// Get the current bounding box (from the latest tracked detection or Kalman filter)
    pub fn current_bbox(&self) -> BBox {
        if let Some(last_detection) = self.track.last() {
            last_detection.bbox.clone()
        } else {
            // Fallback to Kalman filter's current state
            self.kalman_filter.current_bbox()
        }
    }

    /// Get the latest detection
    pub fn latest_detection(&self) -> Option<&Detection> {
        self.track.last().map(|td| &td.detection)
    }

    /// Get the latest detection index
    pub fn latest_detection_index(&self) -> Option<usize> {
        self.track.last().map(|td| td.detection_index)
    }
}
