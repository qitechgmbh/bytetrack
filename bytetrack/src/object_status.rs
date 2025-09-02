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
