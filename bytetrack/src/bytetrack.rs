use std::collections::HashMap;

use crate::{
    bbox::{BBox, build_iou_matrix},
    detection::Detection,
    matching::MatchingAlgorithm,
    object::Object,
};

/// Main ByteTrack tracker implementation
///
/// ByteTrack is a multi-object tracking algorithm that associates detections with
/// existing tracks using a two-stage approach:
/// 1. High-confidence detections are matched to existing tracks using IoU
/// 2. Low-confidence detections are used to recover potentially lost tracks
///
/// The tracker maintains objects with unique IDs and uses Kalman filtering
/// for motion prediction between frames.
///
/// # Type Parameters
/// * `ID` - Type used for unique object identifiers (must be hashable and cloneable)
pub struct Bytetrack<ID>
where
    ID: Eq + std::hash::Hash + Clone,
{
    /// Map of tracked objects indexed by their unique IDs
    objects: HashMap<ID, Object<ID>>,
    /// Configuration parameters for the tracker
    config: BytetrackConfig<ID>,
    /// Last used ID for generating new object IDs
    last_id: Option<ID>,
    /// Current frame index
    last_frame_index: usize,
}

/// Configuration parameters for ByteTrack
///
/// This struct contains all the tunable parameters that control the behavior
/// of the ByteTrack algorithm.
///
/// # Type Parameters
/// * `ID` - Type used for unique object identifiers
pub struct BytetrackConfig<ID>
where
    ID: Eq + std::hash::Hash + Clone,
{
    /// Maximum number of frames an object can be missing before it is removed
    max_disappeared: u32,
    /// Minimum IoU threshold for accepting a track-detection match
    min_iou: f32,
    /// High detection confidence threshold (for primary matching stage)
    high_thresh: f32,
    /// Low detection confidence threshold (for recovery matching stage)
    low_thresh: f32,
    /// Algorithm to use for solving the assignment problem
    algorithm: MatchingAlgorithm,
    /// Function to generate new object IDs
    generate_id: Box<dyn Fn(Option<&ID>) -> ID>,
}

impl Default for BytetrackConfig<u32> {
    fn default() -> Self {
        BytetrackConfig {
            max_disappeared: 5,
            min_iou: 0.3,
            high_thresh: 0.6,
            low_thresh: 0.3,
            algorithm: MatchingAlgorithm::default(),
            generate_id: Box::new(|last_id| last_id.map_or(0, |id| id + 1)),
        }
    }
}

impl<ID> Bytetrack<ID>
where
    ID: Eq + std::hash::Hash + Clone,
{
    /// Create a new ByteTrack tracker with the given configuration
    ///
    /// # Arguments
    /// * `config` - Configuration parameters for the tracker
    pub fn new(config: BytetrackConfig<ID>) -> Self {
        Bytetrack {
            objects: HashMap::new(),
            config,
            last_id: None,
            last_frame_index: 0,
        }
    }

    /// Track objects in the current frame
    ///
    /// This is the main entry point for the tracking algorithm. It processes
    /// the provided detections and updates the internal state of tracked objects.
    ///
    /// # Arguments
    /// * `detections` - Slice of detection references from the current frame
    ///
    /// # Result
    /// * New objects will eb added to [`Self::objects`]
    /// * Updated objects will have their state modified inside [`Self::objects`]
    /// * The [`BytetrackTrackResult`] will be returned
    ///   * The [`BytetrackTrackResult::new`] field contains IDs of newly created objects
    ///   * The [`BytetrackTrackResult::updated`] field contains IDs of updated objects
    ///   * The [`BytetrackTrackResult::removed`] field contains the removed objects
    ///
    pub fn track(&mut self, detections: &[&Detection]) -> BytetrackTrackResult<ID> {
        // Increment frame counter
        self.last_frame_index += 1;
        // Predict new locations of existing objects
        let predicted_bboxes: Vec<_> = self.objects.iter_mut().map(|obj| obj.1.predict()).collect();

        // Split detections into high and low confidence
        let (high_conf_detections, low_conf_detections) =
            split_detections(detections, self.config.high_thresh, self.config.low_thresh);

        // 2. Match high confidence detections to existing objects
        // 2.1 Build iou matrix 
        let high_conf_bboxes: Vec<BBox> = high_conf_detections
            .iter()
            .map(|(_, d)| d.bbox.clone())
            .collect();
        let predicted_bbox_refs: Vec<&BBox> = predicted_bboxes.iter().collect();
        let high_conf_bbox_refs: Vec<&BBox> = high_conf_bboxes.iter().collect();
        let iou_matrix = build_iou_matrix(
            predicted_bbox_refs.as_slice(),
            high_conf_bbox_refs.as_slice(),
        );

        // 2.2 Solve assignment problem (e.g. using Hungarian algorithm)
        let assignments = self
            .config
            .algorithm
            .solve_assignment(&iou_matrix, self.config.min_iou);

        // 2.3 Update matched objects with new detections
        let mut found_objects = Vec::new();
        let mut found_detections = Vec::<usize>::new(); // contains the index (as in `detections` params ) of found detections
        let mut found_detections_object_ids = Vec::<ID>::new(); // contains the object IDs of found detections (same order as found_detections)
        self.objects
            .iter_mut()
            .zip(assignments.iter()) // asseumes HashMap iteration order matches assignments
            .enumerate()
            // update matched objects and store their indices
            .for_each(
                |(object_i, ((_object_id, object), high_conf_detection_i))| {
                    if let Some(high_conf_detection_i) = high_conf_detection_i {
                        let (detection_i, detection) =
                            &high_conf_detections[*high_conf_detection_i];
                        object
                            .update(detection, *detection_i, self.last_frame_index)
                            .expect("Update failed");
                        found_objects.push(object_i);
                        found_detections.push(*detection_i);
                        found_detections_object_ids.push(object.id.clone());
                    }
                },
            );

        // 3. Match low confidence detections to unmatched objects
        // 3.1 Build iou matrix
        let unmatched_predicted_bboxes: Vec<&BBox> = predicted_bboxes
            .iter()
            .enumerate()
            // avoid already matched objects
            .filter(|(i, _)| !found_objects.contains(i))
            .map(|(_, b)| b)
            .collect();
        let low_conf_bboxes: Vec<&BBox> =
            low_conf_detections.iter().map(|(_, d)| &d.bbox).collect();
        let iou_matrix = build_iou_matrix(
            unmatched_predicted_bboxes.as_slice(),
            low_conf_bboxes.as_slice(),
        );

        // 3.2 Solve assignment problem (e.g. using Hungarian algorithm)
        let assignments = self
            .config
            .algorithm
            .solve_assignment(&iou_matrix, self.config.min_iou);

        // 3.3  Update matched objects with new detections
        let mut new_found_objects = Vec::new();
        self.objects
            .iter_mut()
            .enumerate()
            // avoid already matched objects
            .filter(|(object_i, _)| !found_objects.contains(object_i))
            .zip(assignments.iter())
            // update matched objects and store their indices
            .for_each(|((object_i, (_object_id, object)), low_conf_detection_i)| {
                if let Some(low_conf_detection_i) = low_conf_detection_i {
                    let (detection_i, detection) = &low_conf_detections[*low_conf_detection_i];
                    object
                        .update(detection, *detection_i, self.last_frame_index)
                        .expect("Update failed");
                    new_found_objects.push(object_i);
                    found_detections.push(*detection_i);
                    found_detections_object_ids.push(object.id.clone());
                }
            });
        found_objects.extend(new_found_objects);

        // 4. Increment disappeared counter for unmatched objects
        self.objects
            .iter_mut()
            .enumerate()
            .filter(|(i, _)| !found_objects.contains(i))
            .for_each(|(_i, (_id, object))| object.status.incrment_lost_frames());

        // 5. Remove objects that have disappeared for too long
        let mut removed_objects = Vec::new();
        self.objects.retain(|_id, obj| {
            match obj.status.get_lost_frames() <= self.config.max_disappeared {
                true => true,
                false => {
                    removed_objects.push(obj.clone());
                    false
                }
            }
        });

        // 6. Create new objects for unmatched detections
        let mut new_objects: Vec<(usize, ID)> = Vec::new();
        detections
            .iter()
            .enumerate()
            .filter(|(i, _)| !found_detections.contains(i))
            .for_each(|(detection_index, detection)| {
                let new_id = (self.config.generate_id)(self.last_id.as_ref());
                let new_object = Object::from_detection(
                    new_id.clone(),
                    detection,
                    detection_index,
                    self.last_frame_index,
                );
                self.last_id = Some(new_id.clone());
                self.objects.insert(new_id.clone(), new_object);
                new_objects.push((detection_index, new_id));
            });

        // 8. Prepare result
        BytetrackTrackResult {
            new_objects,
            updated_objects: found_detections
                .iter()
                .zip(found_detections_object_ids.iter())
                .map(|(detection_index, object_id)| (*detection_index, object_id.clone()))
                .collect(),
            removed_objects,
        }
    }

    /// Get a reference to all currently tracked objects
    ///
    /// Returns an iterator over (object_id, object) pairs for all
    /// objects currently being tracked.
    pub fn objects(&self) -> impl Iterator<Item = (&ID, &Object<ID>)> {
        self.objects.iter()
    }

    /// Get the current bounding box for an object by ID
    ///
    /// # Arguments
    /// * `object_id` - The ID of the object to retrieve
    ///
    /// # Returns
    /// Option containing the object's current bounding box if found
    pub fn get_object_bbox(&self, object_id: &ID) -> Option<BBox> {
        self.objects.get(object_id).map(|obj| obj.current_bbox())
    }
}

/// Result of a ByteTrack tracking update operation.
///
/// This structure contains the outcomes of processing detections through the ByteTrack algorithm,
/// categorizing objects into three groups based on their tracking state changes.
///
/// # Fields
///
/// * `new_objects` - Newly detected objects that have been assigned tracking IDs for the first time.
///   Each tuple contains the detection index from the input array and the newly assigned object ID.
///
/// * `updated_objects` - Previously tracked objects that have been matched with new detections.
///   Each tuple contains the detection index from the input array and the existing object ID.
///
/// * `removed_objects` - Objects that were previously tracked but are no longer detected or have
///   been determined to be lost. Contains the complete Object instances that were removed.
pub struct BytetrackTrackResult<ID>
where
    ID: Eq + std::hash::Hash + Clone,
{
    /// New objects
    /// * 0: index of the detection in the input detections array
    /// * 1: the created Object Id
    pub new_objects: Vec<(usize, ID)>,

    /// Updated objects
    /// * 0: index of the detection in the input detections array
    /// * 1: the Object Id
    pub updated_objects: Vec<(usize, ID)>,

    /// Removed objects
    pub removed_objects: Vec<Object<ID>>,
}

/// Split detections into high and low confidence based on thresholds
///
/// This function categorizes detections into two groups based on confidence scores:
/// - High confidence: Used for primary track-detection matching
/// - Low confidence: Used for recovering potentially lost tracks
///
/// # Arguments
/// * `detections` - Slice of detection references to categorize
/// * `high_thresh` - Minimum confidence for high-confidence group
/// * `low_thresh` - Minimum confidence for low-confidence group
///
/// # Returns
/// Tuple of (high_confidence_detections, low_confidence_detections) where each
/// vector contains (original_index, cloned_detection) pairs
fn split_detections(
    detections: &[&Detection],
    high_thresh: f32,
    low_thresh: f32,
) -> (Vec<(usize, Detection)>, Vec<(usize, Detection)>) {
    let high_conf_detections = detections
        .iter()
        .enumerate()
        .filter(|(_, detection)| detection.confidence >= high_thresh)
        .map(|(i, detection)| (i, (*detection).clone()))
        .collect();

    let low_conf_detections = detections
        .iter()
        .enumerate()
        .filter(|(_, detection)| {
            detection.confidence >= low_thresh && detection.confidence < high_thresh
        })
        .map(|(i, detection)| (i, (*detection).clone()))
        .collect();

    (high_conf_detections, low_conf_detections)
}

#[cfg(test)]
mod tests {
    use crate::{bbox, det, detection::Detection};

    #[test]
    fn test_split_detections() {
        let detections = vec![
            det!(bbox!(0.0, 0.0, 10.0, 10.0), 0.9),
            det!(bbox!(20.0, 20.0, 10.0, 10.0), 0.5),
            det!(bbox!(40.0, 40.0, 10.0, 10.0), 0.2),
        ];

        let detection_refs: Vec<&Detection> = detections.iter().collect();
        let (high, low) = super::split_detections(&detection_refs, 0.6, 0.3);

        assert_eq!(high[0].0, 0);
        assert_eq!(high[0].1.confidence, 0.9);
        assert_eq!(low[0].0, 1);
        assert_eq!(low[0].1.confidence, 0.5);
        assert_eq!(low.len(), 1);
        assert_eq!(high.len(), 1);
    }

    #[test]
    fn test_build_cost_matrix() {
        let bbox1 = bbox!(0.0, 0.0, 10.0, 10.0);
        let bbox2 = bbox!(5.0, 5.0, 10.0, 10.0);
        let bbox3 = bbox!(20.0, 20.0, 10.0, 10.0);
        let bbox4 = bbox!(25.0, 25.0, 10.0, 10.0);

        let objects = vec![&bbox1, &bbox3];
        let detections = vec![&bbox2, &bbox4];
        let iou_matrix = super::build_iou_matrix(objects.as_slice(), detections.as_slice());
        assert_eq!(iou_matrix.len(), 2);
        assert_eq!(iou_matrix[0].len(), 2);

        println!("{:?}", iou_matrix);
    }
}
