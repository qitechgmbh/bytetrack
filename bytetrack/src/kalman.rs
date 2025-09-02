//! Kalman filter for bounding box tracking
//!
//! This module provides a Kalman filter implementation specifically designed for tracking
//! bounding box parameters: center coordinates (cx, cy), width, height, and their velocities.

use crate::bbox::BBox;
use kalman_filters::{KalmanFilter, KalmanFilterBuilder};
use nalgebra::{SMatrix, SVector};

/// Configuration parameters for the bounding box Kalman filter
#[derive(Debug, Clone)]
pub struct BBoxKalmanConfig {
    /// Time step for the motion model
    pub dt: f32,
    /// Process noise strength for position/size uncertainty
    pub process_noise: f32,
    /// Measurement noise strength
    pub measurement_noise: f32,
    /// Initial position/size uncertainty
    pub initial_position_variance: f32,
    /// Initial velocity uncertainty  
    pub initial_velocity_variance: f32,
}

impl Default for BBoxKalmanConfig {
    fn default() -> Self {
        Self {
            dt: 1.0,
            process_noise: 1.0,
            measurement_noise: 5.0,
            initial_position_variance: 10.0,
            initial_velocity_variance: 100.0,
        }
    }
}

/// Kalman filter for tracking bounding box parameters
///
/// State vector: [cx, cy, w, h, vx, vy, vw, vh]
/// - cx, cy: center coordinates
/// - w, h: width and height
/// - vx, vy, vw, vh: velocities for each parameter
pub struct BBoxKalmanFilter {
    filter: KalmanFilter<f32>,
    config: BBoxKalmanConfig,
}

impl BBoxKalmanFilter {
    /// Create a new Kalman filter for bounding box tracking
    pub fn new(
        initial_bbox: &BBox,
        config: BBoxKalmanConfig,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let center = initial_bbox.center();
        let width = initial_bbox.translation.vector.x;
        let height = initial_bbox.translation.vector.y;

        // State transition matrix for constant velocity model
        // State: [cx, cy, w, h, vx, vy, vw, vh]
        let f = SMatrix::<f32, 8, 8>::from([
            [1.0, 0.0, 0.0, 0.0, config.dt, 0.0, 0.0, 0.0], // cx = cx + vx * dt
            [0.0, 1.0, 0.0, 0.0, 0.0, config.dt, 0.0, 0.0], // cy = cy + vy * dt
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, config.dt, 0.0], // w = w + vw * dt
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, config.dt], // h = h + vh * dt
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],       // vx = vx
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],       // vy = vy
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],       // vw = vw
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],       // vh = vh
        ]);

        // Initial state: center coordinates, width, height, zero velocities
        let x0 = SVector::<f32, 8>::from([
            center.x, center.y, width, height, 0.0, // vx
            0.0, // vy
            0.0, // vw
            0.0, // vh
        ]);

        // Initial covariance - higher uncertainty for velocities
        let p0 = SMatrix::<f32, 8, 8>::from([
            [
                config.initial_position_variance,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                config.initial_position_variance,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                config.initial_position_variance,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                0.0,
                config.initial_position_variance,
                0.0,
                0.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                0.0,
                0.0,
                config.initial_velocity_variance,
                0.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                config.initial_velocity_variance,
                0.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                config.initial_velocity_variance,
                0.0,
            ],
            [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                config.initial_velocity_variance,
            ],
        ]);

        // Process noise (uncertainty in acceleration)
        let dt = config.dt;
        let q_val = config.process_noise;
        let q = SMatrix::<f32, 8, 8>::from([
            [
                q_val * dt * dt * dt / 3.0,
                0.0,
                0.0,
                0.0,
                q_val * dt * dt / 2.0,
                0.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                q_val * dt * dt * dt / 3.0,
                0.0,
                0.0,
                0.0,
                q_val * dt * dt / 2.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                q_val * dt * dt * dt / 3.0,
                0.0,
                0.0,
                0.0,
                q_val * dt * dt / 2.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                0.0,
                q_val * dt * dt * dt / 3.0,
                0.0,
                0.0,
                0.0,
                q_val * dt * dt / 2.0,
            ],
            [
                q_val * dt * dt / 2.0,
                0.0,
                0.0,
                0.0,
                q_val * dt,
                0.0,
                0.0,
                0.0,
            ],
            [
                0.0,
                q_val * dt * dt / 2.0,
                0.0,
                0.0,
                0.0,
                q_val * dt,
                0.0,
                0.0,
            ],
            [
                0.0,
                0.0,
                q_val * dt * dt / 2.0,
                0.0,
                0.0,
                0.0,
                q_val * dt,
                0.0,
            ],
            [
                0.0,
                0.0,
                0.0,
                q_val * dt * dt / 2.0,
                0.0,
                0.0,
                0.0,
                q_val * dt,
            ],
        ]);

        // Observation matrix (we measure cx, cy, w, h)
        let h = SMatrix::<f32, 4, 8>::from_row_slice(&[
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // Measure cx
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // Measure cy
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // Measure w
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, // Measure h
        ]);

        // Measurement noise
        let r_val = config.measurement_noise;
        let r = SMatrix::<f32, 4, 4>::from([
            [r_val, 0.0, 0.0, 0.0],
            [0.0, r_val, 0.0, 0.0],
            [0.0, 0.0, r_val, 0.0],
            [0.0, 0.0, 0.0, r_val],
        ]);

        // Convert SMatrix/SVector to Vec for KalmanFilterBuilder
        let f_vec: Vec<f32> = f.as_slice().to_vec();
        let x0_vec: Vec<f32> = x0.as_slice().to_vec();
        let p0_vec: Vec<f32> = p0.as_slice().to_vec();
        let q_vec: Vec<f32> = q.as_slice().to_vec();
        let h_vec: Vec<f32> = h.as_slice().to_vec();
        let r_vec: Vec<f32> = r.as_slice().to_vec();

        // Build the Kalman filter
        let filter = KalmanFilterBuilder::<f32>::new(8, 4)
            .initial_state(x0_vec)
            .initial_covariance(p0_vec)
            .transition_matrix(f_vec)
            .process_noise(q_vec)
            .observation_matrix(h_vec)
            .measurement_noise(r_vec)
            .build()?;

        Ok(Self { filter, config })
    }

    /// Predict the next bounding box state
    pub fn predict(&mut self) -> BBox {
        // Perform Kalman filter prediction step
        self.filter.predict();

        // Get predicted state [cx, cy, w, h, vx, vy, vw, vh]
        let state = self.filter.state();

        // Create and return predicted bbox
        BBox::from_ccwh(
            state[0], // center x
            state[1], // center y
            state[2], // width
            state[3], // height
        )
    }

    /// Update the filter with a new bounding box measurement
    pub fn update(&mut self, bbox: &BBox) -> Result<(), Box<dyn std::error::Error>> {
        let center = bbox.center();
        let width = bbox.translation.vector.x;
        let height = bbox.translation.vector.y;

        // Create measurement vector [cx, cy, w, h]
        let measurement = vec![center.x, center.y, width, height];

        // Update Kalman filter with measurement
        self.filter.update(&measurement)?;
        Ok(())
    }

    /// Get the current estimated bounding box
    pub fn current_bbox(&self) -> BBox {
        let state = self.filter.state();
        BBox::from_ccwh(
            state[0], // center x
            state[1], // center y
            state[2], // width
            state[3], // height
        )
    }

    /// Get the current state vector [cx, cy, w, h, vx, vy, vw, vh]
    pub fn state(&self) -> &[f32] {
        self.filter.state()
    }

    /// Get the current covariance matrix
    pub fn covariance(&self) -> &[f32] {
        self.filter.covariance()
    }

    /// Get the configuration
    pub fn config(&self) -> &BBoxKalmanConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bbox_kalman_filter_creation() {
        let bbox = BBox::from_ccwh(100.0, 100.0, 50.0, 30.0);
        let config = BBoxKalmanConfig::default();
        let filter = BBoxKalmanFilter::new(&bbox, config);
        assert!(filter.is_ok());
    }

    #[test]
    fn test_predict_and_update() {
        let initial_bbox = BBox::from_ccwh(100.0, 100.0, 50.0, 30.0);
        let config = BBoxKalmanConfig::default();
        let mut filter = BBoxKalmanFilter::new(&initial_bbox, config).unwrap();

        // Predict should work
        let predicted = filter.predict();
        assert_eq!(predicted.center().x, 100.0);
        assert_eq!(predicted.center().y, 100.0);

        // Update should work
        let new_bbox = BBox::from_ccwh(105.0, 103.0, 52.0, 31.0);
        let result = filter.update(&new_bbox);
        assert!(result.is_ok());
    }
}
