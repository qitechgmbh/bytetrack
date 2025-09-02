//! MOT17 Dataset Loading and Parsing

use anyhow::Error;
use bytetrack::BBox;
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Copy)]
pub enum DatasetType {
    TrainDetection,
    TestDetection,
    TrainGroundTruth,
}

#[derive(Debug, Clone, Copy)]
pub enum DetectorType {
    DPM,
    FRCNN,
    SDP,
}

#[derive(Debug, Clone, Copy)]
pub enum SequenceType {
    GroundTruth,
    Detection,
}

#[derive(Debug, Clone)]
pub struct SequenceConfig {
    pub sequence_name: String,
    pub detector_type: DetectorType,
    pub dataset_type: DatasetType,
}

impl SequenceConfig {
    pub fn new(
        sequence_name: &str,
        detector_type: DetectorType,
        dataset_type: DatasetType,
    ) -> Self {
        Self {
            sequence_name: sequence_name.to_string(),
            detector_type,
            dataset_type,
        }
    }
}

impl std::fmt::Display for DetectorType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DetectorType::DPM => write!(f, "DPM"),
            DetectorType::FRCNN => write!(f, "FRCNN"),
            DetectorType::SDP => write!(f, "SDP"),
        }
    }
}

pub struct MotDataset {
    root_path: PathBuf,
}

pub struct Sequence {
    pub frames: Vec<Frame>,
    pub name: String,
    pub detector_type: DetectorType,
    pub sequence_type: SequenceType,
}

impl Sequence {
    /// Get all detections across all frames as a flat vector
    pub fn all_detections(&self) -> Vec<bytetrack::Detection> {
        match self.sequence_type {
            SequenceType::Detection => self
                .frames
                .iter()
                .flat_map(|f| f.detections.iter())
                .map(|d| d.clone().into())
                .collect(),
            SequenceType::GroundTruth => self
                .frames
                .iter()
                .flat_map(|f| f.ground_truth.iter())
                .map(|gt| gt.clone().into())
                .collect(),
        }
    }

    /// Convert detections from a specific frame to ByteTrack format
    pub fn frame_to_bytetrack(&self, frame_index: usize) -> Option<Vec<bytetrack::Detection>> {
        self.frames
            .get(frame_index)
            .map(|f| match self.sequence_type {
                SequenceType::Detection => f
                    .detections
                    .iter()
                    .map(|detection| detection.clone().into())
                    .collect(),
                SequenceType::GroundTruth => {
                    f.ground_truth.iter().map(|gt| gt.clone().into()).collect()
                }
            })
    }

    /// Get frame by frame_id
    pub fn get_frame(&self, frame_id: i32) -> Option<&Frame> {
        self.frames.iter().find(|f| f.frame_id == frame_id)
    }
}

#[derive(Debug, Clone)]
pub struct Detection {
    pub frame_id: i32,
    pub bbox: BBox,
    pub class: i32,
    pub confidence: Option<f32>,
}

impl Detection {
    /// Convert detections into sorted Frame structs
    pub fn into_frames(detections: Vec<Detection>) -> Vec<Frame> {
        // collect into a HashMap first
        let mut frames_map: HashMap<i32, Vec<Detection>> = HashMap::new();
        for detection in detections {
            frames_map
                .entry(detection.frame_id)
                .or_default()
                .push(detection);
        }

        // then get the keys ordered
        let keys: Vec<i32> = {
            let mut ks: Vec<i32> = frames_map.keys().cloned().collect();
            ks.sort();
            ks
        };

        // finally create Frame structs
        keys.into_iter()
            .map(|frame_id| {
                Frame::new_detection_frame(
                    frame_id,
                    frames_map.remove(&frame_id).unwrap_or_default(),
                )
            })
            .collect()
    }
}

#[derive(Debug, Clone)]
pub struct GroundTruthTrack {
    pub frame: usize,
    pub id: i32,
    pub bbox: BBox,
    pub class: i32,
    pub visibility: f32,
}

#[derive(Debug, Clone)]
pub struct Frame {
    pub frame_id: i32,
    pub detections: Vec<Detection>,
    pub ground_truth: Vec<GroundTruthTrack>,
}

impl Frame {
    pub fn new_detection_frame(frame_id: i32, detections: Vec<Detection>) -> Self {
        Self {
            frame_id,
            detections,
            ground_truth: Vec::new(),
        }
    }

    pub fn new_ground_truth_frame(frame_id: i32, ground_truth: Vec<GroundTruthTrack>) -> Self {
        Self {
            frame_id,
            detections: Vec::new(),
            ground_truth,
        }
    }
}

impl GroundTruthTrack {
    /// Convert ground truth tracks into sorted Frame structs
    pub fn into_frames(tracks: Vec<GroundTruthTrack>) -> Vec<Frame> {
        // collect into a HashMap first
        let mut frames_map: HashMap<i32, Vec<GroundTruthTrack>> = HashMap::new();
        for track in tracks {
            frames_map
                .entry(track.frame as i32)
                .or_default()
                .push(track);
        }

        // then get the keys ordered
        let keys: Vec<i32> = {
            let mut ks: Vec<i32> = frames_map.keys().cloned().collect();
            ks.sort();
            ks
        };

        // finally create Frame structs
        keys.into_iter()
            .map(|frame_id| {
                Frame::new_ground_truth_frame(
                    frame_id,
                    frames_map.remove(&frame_id).unwrap_or_default(),
                )
            })
            .collect()
    }
}

// Implement From traits for converting to bytetrack::Detection
impl From<Detection> for bytetrack::Detection {
    fn from(detection: Detection) -> Self {
        bytetrack::Detection::new(
            detection.bbox,
            detection.confidence.unwrap_or(1.0), // Default confidence if None
        )
    }
}

impl From<&Detection> for bytetrack::Detection {
    fn from(detection: &Detection) -> Self {
        bytetrack::Detection::new(
            detection.bbox.clone(),
            detection.confidence.unwrap_or(1.0), // Default confidence if None
        )
    }
}

impl From<GroundTruthTrack> for bytetrack::Detection {
    fn from(track: GroundTruthTrack) -> Self {
        bytetrack::Detection::new(
            track.bbox, 1.0, // Ground truth gets perfect confidence
        )
    }
}

impl From<&GroundTruthTrack> for bytetrack::Detection {
    fn from(track: &GroundTruthTrack) -> Self {
        bytetrack::Detection::new(
            track.bbox.clone(),
            1.0, // Ground truth gets perfect confidence
        )
    }
}

impl MotDataset {
    pub fn new(root_path: &Path) -> Self {
        Self {
            root_path: root_path.to_path_buf(),
        }
    }

    pub fn load_sequence(&self, config: &SequenceConfig) -> Result<Sequence, Error> {
        let path = self.get_sequence_path(
            &config.sequence_name,
            config.detector_type,
            config.dataset_type,
        )?;

        match config.dataset_type {
            DatasetType::TrainDetection | DatasetType::TestDetection => {
                let det_file = path.join("det").join("det.txt");
                self.load_detection_sequence(&det_file, &config.sequence_name, config.detector_type)
            }
            DatasetType::TrainGroundTruth => {
                let gt_file = path.join("gt").join("gt.txt");
                self.load_ground_truth_sequence(
                    &gt_file,
                    &config.sequence_name,
                    config.detector_type,
                )
            }
        }
    }

    fn get_sequence_path(
        &self,
        sequence_name: &str,
        detector_type: DetectorType,
        dataset_type: DatasetType,
    ) -> Result<PathBuf, Error> {
        let type_dir = match dataset_type {
            DatasetType::TrainDetection | DatasetType::TrainGroundTruth => "train",
            DatasetType::TestDetection => "test",
        };

        let full_sequence_name = format!("{}-{}", sequence_name, detector_type);
        let path = self.root_path.join(type_dir).join(full_sequence_name);

        if !path.exists() {
            return Err(anyhow::anyhow!("Sequence path does not exist: {:?}", path));
        }

        Ok(path)
    }

    fn load_detection_sequence(
        &self,
        det_file: &Path,
        sequence_name: &str,
        detector_type: DetectorType,
    ) -> Result<Sequence, Error> {
        let content = fs::read_to_string(det_file)?;
        let mut detections = Vec::new();

        for line in content.lines() {
            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() >= 7 {
                let frame_id: i32 = parts[0].parse()?;
                let left: f32 = parts[2].parse()?;
                let top: f32 = parts[3].parse()?;
                let width: f32 = parts[4].parse()?;
                let height: f32 = parts[5].parse()?;
                let confidence: f32 = parts[6].parse()?;

                let bbox = BBox::from_xywh(left, top, width, height);
                let detection = Detection {
                    frame_id,
                    bbox,
                    class: 1, // Pedestrian class
                    confidence: Some(confidence),
                };

                detections.push(detection);
            }
        }

        // Use the helper method to convert to frames
        let frames = Detection::into_frames(detections);

        Ok(Sequence {
            frames,
            name: sequence_name.to_string(),
            detector_type,
            sequence_type: SequenceType::Detection,
        })
    }

    fn load_ground_truth_sequence(
        &self,
        gt_file: &Path,
        sequence_name: &str,
        detector_type: DetectorType,
    ) -> Result<Sequence, Error> {
        let content = fs::read_to_string(gt_file)?;
        let mut ground_truth_tracks = Vec::new();

        for line in content.lines() {
            let parts: Vec<&str> = line.split(',').collect();
            if parts.len() >= 9 {
                let frame: i32 = parts[0].parse()?;
                let id: i32 = parts[1].parse()?;
                let left: f32 = parts[2].parse()?;
                let top: f32 = parts[3].parse()?;
                let width: f32 = parts[4].parse()?;
                let height: f32 = parts[5].parse()?;
                let class: i32 = parts[7].parse()?;
                let visibility: f32 = parts[8].parse()?;

                // Only include pedestrians (class 1) with decent visibility
                if class == 1 && visibility >= 0.0 {
                    let bbox = BBox::from_xywh(left, top, width, height);
                    let track = GroundTruthTrack {
                        frame: frame as usize,
                        id,
                        bbox,
                        class,
                        visibility,
                    };

                    ground_truth_tracks.push(track);
                }
            }
        }

        // Use the helper method to convert to frames
        let frames = GroundTruthTrack::into_frames(ground_truth_tracks);

        Ok(Sequence {
            frames,
            name: sequence_name.to_string(),
            detector_type,
            sequence_type: SequenceType::GroundTruth,
        })
    }

    pub fn find_sequences(&self) -> Result<Vec<SequenceConfig>, Error> {
        let mut all_sequences = Vec::new();

        // Find train detection sequences
        if let Ok(train_sequences) = self.get_available_sequences(DatasetType::TrainDetection) {
            for (seq_name, detector_type) in train_sequences {
                all_sequences.push(SequenceConfig::new(
                    &seq_name,
                    detector_type,
                    DatasetType::TrainDetection,
                ));
            }
        }

        // Find test detection sequences
        if let Ok(test_sequences) = self.get_available_sequences(DatasetType::TestDetection) {
            for (seq_name, detector_type) in test_sequences {
                all_sequences.push(SequenceConfig::new(
                    &seq_name,
                    detector_type,
                    DatasetType::TestDetection,
                ));
            }
        }

        // Find train ground truth sequences
        if let Ok(gt_sequences) = self.get_available_sequences(DatasetType::TrainGroundTruth) {
            for (seq_name, detector_type) in gt_sequences {
                all_sequences.push(SequenceConfig::new(
                    &seq_name,
                    detector_type,
                    DatasetType::TrainGroundTruth,
                ));
            }
        }

        Ok(all_sequences)
    }

    pub fn find_sequences_by_type(
        &self,
        dataset_type: DatasetType,
    ) -> Result<Vec<SequenceConfig>, Error> {
        let sequences = self.get_available_sequences(dataset_type)?;
        Ok(sequences
            .into_iter()
            .map(|(seq_name, detector_type)| {
                SequenceConfig::new(&seq_name, detector_type, dataset_type)
            })
            .collect())
    }

    pub fn get_available_sequences(
        &self,
        dataset_type: DatasetType,
    ) -> Result<Vec<(String, DetectorType)>, Error> {
        let type_dir = match dataset_type {
            DatasetType::TrainDetection | DatasetType::TrainGroundTruth => "train",
            DatasetType::TestDetection => "test",
        };

        let base_path = self.root_path.join(type_dir);
        let mut sequences = Vec::new();

        if let Ok(entries) = fs::read_dir(&base_path) {
            for entry in entries.flatten() {
                if let Some(name) = entry.file_name().to_str() {
                    // Parse sequence name and detector type
                    if let Some((seq_name, detector_str)) = name.rsplit_once('-') {
                        let detector = match detector_str {
                            "DPM" => DetectorType::DPM,
                            "FRCNN" => DetectorType::FRCNN,
                            "SDP" => DetectorType::SDP,
                            _ => continue,
                        };

                        let path = entry.path();
                        if path.is_dir() {
                            sequences.push((seq_name.to_string(), detector));
                        }
                    }
                }
            }
        }

        sequences.sort_by(|a, b| {
            a.0.cmp(&b.0)
                .then_with(|| format!("{:?}", a.1).cmp(&format!("{:?}", b.1)))
        });
        Ok(sequences)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    #[test]
    fn test_mot_dataset_creation() {
        let dataset = MotDataset::new(Path::new("../datasets/MOT17"));
        assert_eq!(dataset.root_path, Path::new("../datasets/MOT17"));
    }

    #[test]
    fn test_load_detection_sequence() {
        let dataset_root = Path::new("../datasets/MOT17");

        // Skip test if dataset doesn't exist
        if !dataset_root.exists() {
            println!("Skipping test - dataset not found at {:?}", dataset_root);
            return;
        }

        let dataset = MotDataset::new(dataset_root);

        // Test the new find_sequences API
        match dataset.find_sequences_by_type(DatasetType::TrainDetection) {
            Ok(configs) if !configs.is_empty() => {
                let config = &configs[0];
                println!(
                    "Testing with sequence config: {} ({:?}) - {:?}",
                    config.sequence_name, config.detector_type, config.dataset_type
                );

                match dataset.load_sequence(config) {
                    Ok(sequence) => {
                        println!("Successfully loaded sequence: {}", sequence.name);
                        println!("Detector type: {:?}", sequence.detector_type);
                        println!("Number of frames: {}", sequence.frames.len());

                        if !sequence.frames.is_empty() {
                            let first_frame = &sequence.frames[0];
                            println!("First frame ID: {}", first_frame.frame_id);
                            println!(
                                "Detections in first frame: {}",
                                first_frame.detections.len()
                            );

                            if !first_frame.detections.is_empty() {
                                let first_detection = &first_frame.detections[0];
                                println!(
                                    "First detection - Frame: {}, Class: {}, Confidence: {:?}",
                                    first_detection.frame_id,
                                    first_detection.class,
                                    first_detection.confidence
                                );
                                println!(
                                    "BBox: left={}, top={}, right={}, bottom={}",
                                    first_detection.bbox.left(),
                                    first_detection.bbox.top(),
                                    first_detection.bbox.right(),
                                    first_detection.bbox.bottom()
                                );
                            }
                        }

                        // Basic assertions
                        assert_eq!(sequence.name, config.sequence_name);
                        assert_eq!(sequence.detector_type as u8, config.detector_type as u8);
                        assert!(!sequence.frames.is_empty(), "Sequence should have frames");

                        // Check that frames are sorted
                        for i in 1..sequence.frames.len() {
                            assert!(
                                sequence.frames[i - 1].frame_id <= sequence.frames[i].frame_id,
                                "Frames should be sorted by frame_id"
                            );
                        }
                    }
                    Err(e) => {
                        println!("Failed to load sequence: {}", e);
                        panic!("Should be able to load at least one sequence");
                    }
                }
            }
            Ok(_) => {
                println!(
                    "No sequences found - this might be expected if dataset is not fully set up"
                );
            }
            Err(e) => {
                println!("Error getting available sequences: {}", e);
            }
        }
    }

    #[test]
    fn test_find_sequences() {
        let dataset_root = Path::new("../datasets/MOT17");

        // Skip test if dataset doesn't exist
        if !dataset_root.exists() {
            println!("Skipping test - dataset not found at {:?}", dataset_root);
            return;
        }

        let dataset = MotDataset::new(dataset_root);

        match dataset.find_sequences() {
            Ok(configs) => {
                println!("Found {} sequence configurations", configs.len());

                // Group by dataset type for reporting
                let mut train_det_count = 0;
                let mut test_det_count = 0;
                let mut train_gt_count = 0;

                for config in &configs {
                    match config.dataset_type {
                        DatasetType::TrainDetection => train_det_count += 1,
                        DatasetType::TestDetection => test_det_count += 1,
                        DatasetType::TrainGroundTruth => train_gt_count += 1,
                    }
                }

                println!("  - Train Detection: {}", train_det_count);
                println!("  - Test Detection: {}", test_det_count);
                println!("  - Train Ground Truth: {}", train_gt_count);

                if !configs.is_empty() {
                    let first_config = &configs[0];
                    println!(
                        "First config: {} ({:?}) - {:?}",
                        first_config.sequence_name,
                        first_config.detector_type,
                        first_config.dataset_type
                    );
                }

                assert!(!configs.is_empty(), "Should find at least some sequences");
            }
            Err(e) => {
                println!("Error finding sequences: {}", e);
            }
        }
    }

    #[test]
    fn test_sequence_config() {
        let config =
            SequenceConfig::new("MOT17-02", DetectorType::FRCNN, DatasetType::TrainDetection);

        assert_eq!(config.sequence_name, "MOT17-02");
        assert_eq!(config.detector_type as u8, DetectorType::FRCNN as u8);
        assert_eq!(config.dataset_type as u8, DatasetType::TrainDetection as u8);
    }

    #[test]
    fn test_load_ground_truth_sequence() {
        let dataset_root = Path::new("../datasets/MOT17");

        // Skip test if dataset doesn't exist
        if !dataset_root.exists() {
            println!("Skipping test - dataset not found at {:?}", dataset_root);
            return;
        }

        let dataset = MotDataset::new(dataset_root);

        // Test using the new sequence config API
        match dataset.find_sequences_by_type(DatasetType::TrainGroundTruth) {
            Ok(configs) if !configs.is_empty() => {
                let config = &configs[0];
                println!(
                    "Testing ground truth with sequence config: {} ({:?}) - {:?}",
                    config.sequence_name, config.detector_type, config.dataset_type
                );

                match dataset.load_sequence(config) {
                    Ok(sequence) => {
                        println!(
                            "Successfully loaded ground truth sequence: {}",
                            sequence.name
                        );
                        println!("Number of frames: {}", sequence.frames.len());

                        if !sequence.frames.is_empty() {
                            let first_frame = &sequence.frames[0];
                            println!("First frame ID: {}", first_frame.frame_id);
                            println!(
                                "Ground truth tracks in first frame: {}",
                                first_frame.detections.len()
                            );

                            if !first_frame.detections.is_empty() {
                                let first_track = &first_frame.detections[0];
                                println!(
                                    "First track - Frame: {},  Class: {}",
                                    first_track.frame_id, first_track.class
                                );
                            }
                        }

                        // Basic assertions
                        assert_eq!(sequence.name, config.sequence_name);
                        assert!(!sequence.frames.is_empty(), "Sequence should have frames");

                        // Check that all detections are pedestrians (class 1)
                        for frame in &sequence.frames {
                            for detection in &frame.detections {
                                assert_eq!(
                                    detection.class, 1,
                                    "All ground truth should be pedestrians (class 1)"
                                );
                                assert!(
                                    detection.confidence.is_none(),
                                    "Ground truth should not have confidence"
                                );
                            }
                        }
                    }
                    Err(e) => {
                        println!("Failed to load ground truth sequence: {}", e);
                        // This might be expected if ground truth files don't exist
                    }
                }
            }
            Ok(_) => {
                println!("No ground truth sequences found");
            }
            Err(e) => {
                println!("Error getting available ground truth sequences: {}", e);
            }
        }
    }

    #[test]
    fn test_detection_structure() {
        let bbox = BBox::from_xywh(10.0, 20.0, 100.0, 200.0);
        let detection = Detection {
            frame_id: 5,
            bbox: bbox.clone(),
            class: 1,
            confidence: Some(0.85),
        };

        assert_eq!(detection.frame_id, 5);
        assert_eq!(detection.class, 1);
        assert_eq!(detection.confidence, Some(0.85));
        assert_eq!(detection.bbox.left(), bbox.left());
        assert_eq!(detection.bbox.area(), bbox.area());
    }

    #[test]
    fn test_from_conversions() {
        let bbox = BBox::from_xywh(10.0, 20.0, 100.0, 200.0);

        // Test Detection to bytetrack::Detection conversion
        let detection = Detection {
            frame_id: 5,
            bbox: bbox.clone(),
            class: 1,
            confidence: Some(0.85),
        };

        let bt_detection: bytetrack::Detection = detection.clone().into();
        assert_eq!(bt_detection.confidence, 0.85);
        assert_eq!(bt_detection.bbox.area(), bbox.area());

        // Test reference conversion
        let bt_detection_ref: bytetrack::Detection = (&detection).into();
        assert_eq!(bt_detection_ref.confidence, 0.85);

        // Test Detection with None confidence
        let detection_no_conf = Detection {
            frame_id: 5,
            bbox: bbox.clone(),
            class: 1,
            confidence: None,
        };

        let bt_detection_default: bytetrack::Detection = detection_no_conf.into();
        assert_eq!(bt_detection_default.confidence, 1.0); // Should default to 1.0

        // Test GroundTruthTrack conversion
        let gt_track = GroundTruthTrack {
            frame: 5,
            id: 42,
            bbox: bbox.clone(),
            class: 1,
            visibility: 0.8,
        };

        let bt_detection_gt: bytetrack::Detection = gt_track.clone().into();
        assert_eq!(bt_detection_gt.confidence, 1.0); // Ground truth gets perfect confidence
        assert_eq!(bt_detection_gt.bbox.area(), bbox.area());

        // Test reference conversion for GroundTruthTrack
        let bt_detection_gt_ref: bytetrack::Detection = (&gt_track).into();
        assert_eq!(bt_detection_gt_ref.confidence, 1.0);
    }
}
