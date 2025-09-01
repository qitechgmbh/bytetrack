use crate::bbox::BBox;

/// Raw detection by the object detector
#[derive(Clone, Debug)]
pub struct Detection {
    /// Bounding box of the detection
    pub bbox: BBox,
    /// Confidence score of the detection
    pub confidence: f32,
}

impl Detection {
    /// Create a new detection
    pub fn new(bbox: BBox, confidence: f32) -> Self {
        Detection { bbox, confidence }
    }
}

#[macro_export]
macro_rules! det {
    ($point:expr, $conf:expr) => {
        Detection::new($point, $conf)
    };
}
