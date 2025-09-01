use std::{collections::HashMap, hash::Hash};

use crate::{
    bbox::build_iou_matrix, detection::Detection, matching::MatchingAlgorithm, object::Object,
};

pub struct Bytetrack<T: Eq + Hash> {
    objects: HashMap<T, Object>,
    config: BytetrackConfig<T>,
}

pub struct BytetrackConfig<ID: Eq + Hash> {
    // Maximum number of frames an object can be missing before it is removed
    max_disappeared: u32,
    /// Maximum distance between two objects to be considered the same
    min_iou: f32,
    /// High detection confidence threshold
    high_thresh: f32,
    /// Low detection confidence threshold
    low_thresh: f32,
    /// Algorithm to use for matching
    algorithm: MatchingAlgorithm,
    /// new object ID generator
    generate_id: Box<dyn Fn(Option<&ID>) -> ID>,
    /// last used ID
    last_id: Option<ID>,
}

impl Default for BytetrackConfig<usize> {
    fn default() -> Self {
        BytetrackConfig {
            max_disappeared: 5,
            min_iou: 0.3,
            high_thresh: 0.6,
            low_thresh: 0.3,
            algorithm: MatchingAlgorithm::default(),
            generate_id: Box::new(|last_id| last_id.map_or(0, |id| id + 1)),
            last_id: None,
        }
    }
}

impl<ID: Eq + Hash> Bytetrack<ID> {
    pub fn new(config: BytetrackConfig<ID>) -> Self {
        Bytetrack {
            objects: HashMap::new(),
            config,
        }
    }

    pub fn track<'a>(&mut self, detections: impl Iterator<Item = &'a Detection> + Clone) -> () {
        // Predict new locations of existing objects
        let predicted_bboxes: Vec<_> = self.objects.iter_mut().map(|obj| obj.1.predict()).collect();

        // Split detections into high and low confidence
        let (high_conf_detections, low_conf_detections) = split_detections(
            detections.clone(),
            self.config.high_thresh,
            self.config.low_thresh,
        );

        // 2. Match high confidence detections to existing objects
        // 2.1 Build cost matrix (e.g. using IoU)
        let iou_matrix = build_iou_matrix(
            predicted_bboxes.iter(),
            high_conf_detections.iter().map(|(_, d)| &d.bbox),
        );

        // 2.2 Solve assignment problem (e.g. using Hungarian algorithm)
        // Vec<Option<usize>> Track-to-detection mapping so usize is a detection index
        let assignments = self
            .config
            .algorithm
            .solve_assignment(&iou_matrix, self.config.min_iou);

        // 2.3 Update matched objects with new detections
        let mut found_objects = Vec::new();
        let mut found_detections = Vec::<usize>::new(); // contains the index (as in `detections` params ) of found detections
        self.objects
            .iter_mut()
            .zip(assignments.iter())
            .enumerate()
            // update matched objects and store their indices
            .for_each(
                |(object_i, ((_object_id, object), high_conf_detection_i))| {
                    if let Some(high_conf_detection_i) = high_conf_detection_i {
                        let (detection_i, detection) =
                            &high_conf_detections[*high_conf_detection_i];
                        object.update(detection.clone()).expect("Update failed");
                        found_objects.push(object_i);
                        found_detections.push(*detection_i);
                    }
                },
            );

        // 4. Match low confidence detections to unmatched objects
        // 4.1 Build cost matrix (e.g. using IoU)
        let iou_matrix = build_iou_matrix(
            predicted_bboxes
                .iter()
                .enumerate()
                // avaoid already matched objects
                .filter(|(i, _)| !found_objects.contains(i))
                .map(|(_, b)| b),
            low_conf_detections.iter().map(|(_, d)| &d.bbox),
        );

        // 4.2 Solve assignment problem (e.g. using Hungarian algorithm)
        let assignments = self
            .config
            .algorithm
            .solve_assignment(&iou_matrix, self.config.min_iou);

        // 4.3  Update matched objects with new detections
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
                    object.update(detection.clone()).expect("Update failed");
                    new_found_objects.push(object_i);
                    found_detections.push(*detection_i);
                }
            });
        found_objects.extend(new_found_objects);

        // 6. Increment disappeared counter for unmatched objects
        self.objects
            .iter_mut()
            .enumerate()
            .filter(|(i, _)| !found_objects.contains(i))
            .for_each(|(_i, (_id, object))| object.status.incrment_lost_frames());

        // 7. Remove objects that have disappeared for too long
        self.objects
            .retain(|_id, obj| obj.status.get_lost_frames() <= self.config.max_disappeared);

        // 8. Create new objects for unmatched detections
        detections
            .enumerate()
            .filter(|(i, _)| !found_detections.contains(i))
            .for_each(|(_i, detection)| {
                let new_id = (self.config.generate_id)(self.config.last_id.as_ref());
                let new_object = Object::from_detection((*detection).clone());
                self.objects.insert(new_id, new_object);
            });
    }
}

/// Split detections into high and low confidence based on thresholds
///
/// Returns two vectors containing the original index and the detection
fn split_detections<'a>(
    detections: impl Iterator<Item = &'a Detection> + Clone,
    high_thresh: f32,
    low_thresh: f32,
) -> (Vec<(usize, Detection)>, Vec<(usize, Detection)>) {
    let high_conf_detections = detections
        .clone()
        .enumerate()
        .filter(|(_, detection)| detection.confidence >= high_thresh)
        .map(|(i, detection)| (i, detection.clone()))
        .collect();

    let low_conf_detections = detections
        .enumerate()
        .filter(|(_, detection)| {
            detection.confidence >= low_thresh && detection.confidence < high_thresh
        })
        .map(|(i, detection)| (i, detection.clone()))
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

        let (high, low) = super::split_detections(detections.iter(), 0.6, 0.3);

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
        let iou_matrix = super::build_iou_matrix(objects.into_iter(), detections.into_iter());
        assert_eq!(iou_matrix.len(), 2);
        assert_eq!(iou_matrix[0].len(), 2);

        println!("{:?}", iou_matrix);
    }
}
