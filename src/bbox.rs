use nalgebra::{Point2, Translation2};

#[derive(Clone, Debug)]
pub struct BBox {
    pub origin: Point2<f32>,
    pub translation: Translation2<f32>, // width = x, height = y
}

impl BBox {
    pub fn new(origin: Point2<f32>, width: f32, height: f32) -> Self {
        BBox {
            origin,
            translation: Translation2::new(width, height),
        }
    }

    /// Create a bounding box from (x, y, width, height)
    pub fn from_xywh(x: f32, y: f32, width: f32, height: f32) -> Self {
        BBox {
            origin: Point2::new(x, y),
            translation: Translation2::new(width, height),
        }
    }

    /// Create a bounding box from (x1, y1, x2, y2)
    pub fn from_xyxy(x1: f32, y1: f32, x2: f32, y2: f32) -> Self {
        let width = x2 - x1.clone();
        let height = y2 - y1.clone();
        Self::from_xywh(x1, y1, width, height)
    }

    /// Create a bounding box from (center_x, center_y, width, height)
    pub fn from_ccwh(cx: f32, cy: f32, w: f32, h: f32) -> Self {
        let x = cx - w / 2.0;
        let y = cy - h / 2.0;
        Self::from_xywh(x, y, w, h)
    }

    #[inline]
    fn width(&self) -> f32 {
        self.translation.vector.x.clone()
    }

    #[inline]
    fn height(&self) -> f32 {
        self.translation.vector.y.clone()
    }

    #[inline]
    fn right(&self) -> f32 {
        self.origin.x.clone() + self.width()
    }

    #[inline]
    fn bottom(&self) -> f32 {
        self.origin.y.clone() + self.height()
    }

    pub fn iou(&self, other: &BBox) -> f32 {
        // Intersection rectangle extents
        let inter_left = self.origin.x.clone().max(other.origin.x.clone());
        let inter_top = self.origin.y.clone().max(other.origin.y.clone());
        let inter_right = self.right().min(other.right());
        let inter_bottom = self.bottom().min(other.bottom());

        // Compute intersection width/height with clamp to zero
        let inter_w = (inter_right - inter_left).max(0.0);
        let inter_h = (inter_bottom - inter_top).max(0.0);

        // Areas
        let inter_area = inter_w * inter_h;
        let self_area = self.width() * self.height();
        let other_area = other.width() * other.height();

        // Union = sum - intersection
        let union = self_area + other_area - inter_area.clone();

        // Handle degenerate cases: if union == 0, define IoU = 0
        if union <= 0.0 {
            return 0.0;
        }

        inter_area / union
    }

    #[inline]
    pub fn center(&self) -> Point2<f32> {
        Point2::<f32>::new(
            self.origin.x.clone() + self.width() / 2.0,
            self.origin.y.clone() + self.height() / 2.0,
        )
    }

    #[inline]
    pub fn diagonal(&self) -> f32 {
        self.translation.vector.norm()
    }
}

mod tests {

    #[test]
    fn test_iou() {
        // Test cases with known IoU values for validation
        let rect1 = super::BBox::from_xywh(0.0, 0.0, 10.0, 10.0);
        let rect2 = super::BBox::from_xywh(5.0, 5.0, 10.0, 10.0);
        let iou = rect1.iou(&rect2);
        assert!((iou - 0.14285714).abs() < 1e-6);

        let rect3 = super::BBox::from_xywh(10.0, 10.0, 10.0, 10.0);
        let rect4 = super::BBox::from_xywh(20.0, 20.0, 10.0, 10.0);
        assert_eq!(rect3.iou(&rect4), 0.0);

        let rect5 = super::BBox::from_xywh(0.0, 0.0, 20.0, 20.0);
        let rect6 = super::BBox::from_xywh(5.0, 5.0, 10.0, 10.0);
        assert_eq!(rect5.iou(&rect6), 0.25);

        let rect7 = super::BBox::from_xywh(0.0, 0.0, 10.0, 10.0);
        let rect8 = super::BBox::from_xywh(0.0, 0.0, 10.0, 10.0);
        assert_eq!(rect7.iou(&rect8), 1.0);

        // Test case with two rectangles having zero width and height
        let rect9 = super::BBox::from_xywh(0.0, 0.0, 0.0, 0.0);
        let rect10 = super::BBox::from_xywh(5.0, 5.0, 0.0, 0.0);
        assert_eq!(rect9.iou(&rect10), 0.0);

        // Test case with two rectangles are the same
        let rect11 = super::BBox::from_xywh(4.5, 2.0, 10.0, 10.0);
        let rect12 = super::BBox::from_xywh(4.5, 2.0, 10.0, 10.0);
        assert_eq!(rect11.iou(&rect12), 1.0);
    }
}

/// Calls [`Rect::from_xyxy`]
#[macro_export]
macro_rules! bbox_xyxy {
    ($x1:expr, $y1:expr, $x2:expr, $y2:expr) => {
        $crate::bbox::BBox::from_xyxy($x1, $y1, $x2, $y2)
    };
}

/// Calls [`Rect::from_xywh`]
#[macro_export]
macro_rules! bbox {
    ($x:expr, $y:expr, $w:expr, $h:expr) => {
        $crate::bbox::BBox::from_xywh($x, $y, $w, $h)
    };
}

/// Build iou matrix based on IoU between objects and detections
///
/// `row_bboxes` - iterator over object bounding boxes (rows)
/// `column_bboxes` - iterator over detection bounding boxes (columns)
pub fn build_iou_matrix<'a>(
    object_bboxes: impl Iterator<Item = &'a BBox>,
    detection_bboxes: impl Iterator<Item = &'a BBox>,
) -> Vec<Vec<f32>> {
    let object_bboxes: Vec<_> = object_bboxes.collect();
    let detection_bboxes: Vec<_> = detection_bboxes.collect();

    let mut cost_matrix = vec![vec![0.0; detection_bboxes.len()]; object_bboxes.len()];
    for (i, object) in object_bboxes.iter().enumerate() {
        for (j, detection) in detection_bboxes.iter().enumerate() {
            let iou = object.iou(&detection);
            cost_matrix[i][j] = iou; // IoU
        }
    }
    cost_matrix
}
