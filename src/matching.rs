/// Assignment algorithms for matching tracks to detections
///
/// This enum defines different algorithms that can be used to solve the assignment
/// problem in multi-object tracking, where we need to match existing tracks to
/// new detections optimally.
#[derive(Debug, Clone)]
pub enum MatchingAlgorithm {
    /// Hungarian algorithm (Kuhn-Munkres) for optimal assignment
    ///
    /// The scale_factor is used to convert floating-point IoU values to integers
    /// for the integer-based Hungarian algorithm implementation.
    Hungarian { scale_factor: f32 },
    // Greedy,
}

impl Default for MatchingAlgorithm {
    fn default() -> Self {
        MatchingAlgorithm::Hungarian {
            scale_factor: 1000.0,
        }
    }
}

use pathfinding::kuhn_munkres::kuhn_munkres_min;
use pathfinding::matrix::Matrix;

impl MatchingAlgorithm {
    /// Solves the assignment problem to match tracks to detections.
    ///
    /// # Arguments
    /// * `iou_matrix` - IoU similarity matrix where `iou_matrix[i][j]` is the IoU
    ///   between track `i` and detection `j`. Dimensions: `num_tracks × num_detections`
    /// * `iou_min` - Minimum IoU threshold for accepting a match
    ///
    /// # Returns
    /// A **track-to-detection** mapping vector of length `num_tracks` where:
    /// - `result[track_idx] = Some(detection_idx)` means track `track_idx` is matched to detection `detection_idx`
    /// - `result[track_idx] = None` means track `track_idx` is unmatched (no suitable detection found)
    ///
    /// # Example
    /// ```rust
    /// use bytetrack::matching::MatchingAlgorithm;
    ///
    /// let matcher = MatchingAlgorithm::default();
    /// let iou_matrix = vec![
    ///     vec![0.8, 0.2],  // Track 0: high IoU with detection 0, low with detection 1
    ///     vec![0.1, 0.9],  // Track 1: low IoU with detection 0, high with detection 1
    /// ];
    /// let assignments = matcher.solve_assignment(&iou_matrix, 0.5);
    /// // Expected result: [Some(0), Some(1)]
    /// // Track 0 → Detection 0, Track 1 → Detection 1
    /// assert_eq!(assignments, vec![Some(0), Some(1)]);
    /// ```
    ///
    /// # Notes
    /// - If `iou_matrix` is empty, returns an empty vector
    /// - If no detections exist, returns `vec![None; num_tracks]`
    /// - Uses Hungarian algorithm for optimal assignment
    /// - Matches below `iou_min` threshold are rejected (returned as `None`)
    pub fn solve_assignment(&self, iou_matrix: &[Vec<f32>], iou_min: f32) -> Vec<Option<usize>> {
        match self {
            MatchingAlgorithm::Hungarian { scale_factor } => {
                let num_tracks = iou_matrix.len();
                if num_tracks == 0 {
                    return Vec::new();
                }

                // Determine number of detections from the widest row
                let num_detections = iou_matrix[0].len();
                if num_detections == 0 {
                    // No detections at all → all unmatched
                    return vec![None; num_tracks];
                }

                // Ensure rectangularity (debug-time check)
                debug_assert!(
                    iou_matrix.iter().all(|row| row.len() == num_detections),
                    "iou_matrix must be rectangular: each row must have num_detections columns"
                );

                // Hungarian expects square; pad with dummy columns (cost = 1.0 - IoU = 1.0)
                let padded_cols = num_tracks.max(num_detections);

                // Build integer cost matrix row-major
                let mut cost_data = Vec::with_capacity(num_tracks * padded_cols);
                for i in 0..num_tracks {
                    // Safety: if some row is shorter, treat missing as 0 IoU (worst)
                    let row = &iou_matrix[i];
                    for j in 0..padded_cols {
                        let cost = if j < num_detections {
                            let iou = row.get(j).copied().unwrap_or(0.0).clamp(0.0, 1.0);
                            scale(cost(iou), *scale_factor)
                        } else {
                            // Dummy detection: cost of 1.0
                            scale(1.0, *scale_factor)
                        };
                        cost_data.push(cost);
                    }
                }

                // Construct matrix; dimensions are guaranteed, so expect() is fine here
                let cost_matrix = Matrix::from_vec(num_tracks, padded_cols, cost_data)
                    .expect("dimensions guaranteed; construction should not fail");

                // Run Hungarian for minimization
                let (_total_cost, assignments) = kuhn_munkres_min(&cost_matrix);
                // assignments.len() == num_tracks; entries are column indices in [0, padded_cols)

                // Apply acceptance threshold on real (non-dummy) matches
                let max_acceptable_cost = scale(cost(iou_min), *scale_factor);

                let mut result = Vec::with_capacity(num_tracks);
                for (track_i, det_i) in assignments.into_iter().enumerate() {
                    if det_i < num_detections {
                        let cost = cost_matrix[(track_i, det_i)];
                        if cost <= max_acceptable_cost {
                            result.push(Some(det_i));
                        } else {
                            result.push(None);
                        }
                    } else {
                        // Matched to a dummy column
                        result.push(None);
                    }
                }
                result
            }
        }
    }
}

/// Convert IoU to cost (1.0 - IoU) for minimization algorithms
#[inline]
fn cost(iou: f32) -> f32 {
    1.0 - iou
}

/// Scale floating-point cost to integer for Hungarian algorithm
#[inline]
fn scale(value: f32, scale_factor: f32) -> i32 {
    (value * scale_factor).round() as i32
}
