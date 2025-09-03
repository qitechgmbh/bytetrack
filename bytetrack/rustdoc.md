# Crate Documentation

**Version:** 0.1.0

**Format Version:** 43

# Module `bytetrack`

# ByteTrack: Multi-Object Tracking Library

This library provides a Rust port of the [Python library](https://github.com/FoundationVision/ByteTrack) of the [ByteTrack algorithm](https://arxiv.org/abs/2110.06864) for multi-object tracking (MOT) in real-time and computer vision applications. ByteTrack is also in [Multi-Object Tracking with Ultralytics YOLO](https://docs.ultralytics.com/de/modes/track/).

## Features

- Idiomatic API
- Access historical
  - tracked positions
  - detected bounding boxes (your input)
  - predicted bounding boxes (Kalman filter)
  - tracked/lost state
- Configure ByteTrack & Kalman filter parameters
- Customize ID generation (e.g. random UUIDs, incremental u32, etc.)

## Example

```rust
use bytetrack::{Detection, Bytetrack, BytetrackConfig};

fn main() {
    let mut total_tracked_objects = 0;

    let frames: Vec<Vec<Detection>> = // insert frames with detection here

    // Initializing tracker which uses u32 unique incremental IDs for tracked objects
    let mut tracker = Bytetrack::new(BytetrackConfig::<u32>::default());

    // Process each frame
    for (i, frame) in frames.iter().enumerate() {
        println!("Processing frame {} with {} detections", i, frame.len());

        // Track objects in the current frame
        let result = tracker.track(&frame_detections);

        // Count removed objects
        let removed_count = result.removed_objects.len();
        total_tracked_objects += result.new_objects.len();

        // Read tracking results or access tracker.get_objects() directly
        println!("New object detected: {:?}", result.new_objects.len());
        println!("Updated objects: {:?}", result.updated_objects.len());
        println!("Removed objects: {:?}", removed_count);
    }

    // Read final tracked objects and clear internal state
    let final_count = tracker.drain_objects();
    total_tracked_objects += final_count.len();
    println!("Total tracked objects: {:?}", total_tracked_objects);

    // Print track of some object
    if let Some(object) = final_objects.last() {
        println!(" Track of object {}:", object.id);
        for t in object.track.iter() {
            println!(
                "Frame: {}, X: {}, Y: {}",
                t.frame_index,
                t.bbox.center().x,
                t.bbox.center().y
            )
        }
    }
}
```

## Modules

## Module `bbox`

```rust
pub mod bbox { /* ... */ }
```

### Types

#### Struct `BBox`

A 2D bounding box representation using origin point and translation vector

The bounding box is defined by its top-left corner (origin) and dimensions (translation).
This representation allows for efficient geometric operations and IoU calculations.

```rust
pub struct BBox {
    pub origin: nalgebra::Point2<f32>,
    pub translation: nalgebra::Translation2<f32>,
}
```

##### Fields

| Name | Type | Documentation |
|------|------|---------------|
| `origin` | `nalgebra::Point2<f32>` | Top-left corner of the bounding box |
| `translation` | `nalgebra::Translation2<f32>` | Translation vector representing width (x) and height (y) |

##### Implementations

###### Methods

- ```rust
  pub fn new(origin: Point2<f32>, width: f32, height: f32) -> Self { /* ... */ }
  ```
  Create a new bounding box from origin point and dimensions

- ```rust
  pub fn from_xywh(x: f32, y: f32, width: f32, height: f32) -> Self { /* ... */ }
  ```
  Create a bounding box from (x, y, width, height)

- ```rust
  pub fn from_xyxy(x1: f32, y1: f32, x2: f32, y2: f32) -> Self { /* ... */ }
  ```
  Create a bounding box from (x1, y1, x2, y2)

- ```rust
  pub fn from_ccwh(cx: f32, cy: f32, w: f32, h: f32) -> Self { /* ... */ }
  ```
  Create a bounding box from (center_x, center_y, width, height)

- ```rust
  pub fn width(self: &Self) -> f32 { /* ... */ }
  ```
  Get the width of the bounding box

- ```rust
  pub fn height(self: &Self) -> f32 { /* ... */ }
  ```
  Get the height of the bounding box

- ```rust
  pub fn left(self: &Self) -> f32 { /* ... */ }
  ```
  Get the x-coordinate of the left edge (same as origin.x)

- ```rust
  pub fn top(self: &Self) -> f32 { /* ... */ }
  ```
  Get the y-coordinate of the top edge (same as origin.y)

- ```rust
  pub fn right(self: &Self) -> f32 { /* ... */ }
  ```
  Get the x-coordinate of the right edge

- ```rust
  pub fn bottom(self: &Self) -> f32 { /* ... */ }
  ```
  Get the y-coordinate of the bottom edge

- ```rust
  pub fn area(self: &Self) -> f32 { /* ... */ }
  ```
  Get the area of the bounding box

- ```rust
  pub fn iou(self: &Self, other: &BBox) -> f32 { /* ... */ }
  ```
  Calculate Intersection over Union (IoU) with another bounding box

- ```rust
  pub fn center(self: &Self) -> Point2<f32> { /* ... */ }
  ```
  Get the center point of the bounding box

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Clone**
  - ```rust
    fn clone(self: &Self) -> BBox { /* ... */ }
    ```

- **CloneToUninit**
  - ```rust
    unsafe fn clone_to_uninit(self: &Self, dest: *mut u8) { /* ... */ }
    ```

- **Debug**
  - ```rust
    fn fmt(self: &Self, f: &mut $crate::fmt::Formatter<''_>) -> $crate::fmt::Result { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **ToOwned**
  - ```rust
    fn to_owned(self: &Self) -> T { /* ... */ }
    ```

  - ```rust
    fn clone_into(self: &Self, target: &mut T) { /* ... */ }
    ```

- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

### Functions

#### Function `build_iou_matrix`

Build iou matrix based on IoU between objects and detections

`object_bboxes` - slice of object bounding boxes (rows)
`detection_bboxes` - slice of detection bounding boxes (columns)

```rust
pub fn build_iou_matrix(object_bboxes: &[&BBox], detection_bboxes: &[&BBox]) -> Vec<Vec<f32>> { /* ... */ }
```

## Module `bytetrack`

```rust
pub mod bytetrack { /* ... */ }
```

### Types

#### Struct `BytetrackConfig`

Configuration parameters for ByteTrack

This struct contains all the tunable parameters that control the behavior
of the ByteTrack algorithm.

# Type Parameters
* `ID` - Type used for unique object identifiers

```rust
pub struct BytetrackConfig<ID> {
    pub max_disappeared: u32,
    pub min_iou: f32,
    pub high_thresh: f32,
    pub low_thresh: f32,
    pub algorithm: crate::matching::MatchingAlgorithm,
    pub generate_id: std::sync::Arc<dyn Fn(Option<&ID>) -> ID + Send + Sync>,
}
```

##### Fields

| Name | Type | Documentation |
|------|------|---------------|
| `max_disappeared` | `u32` | Maximum number of frames an object can be missing before it is removed |
| `min_iou` | `f32` | Minimum IoU threshold for accepting a track-detection match |
| `high_thresh` | `f32` | High detection confidence threshold (for primary matching stage) |
| `low_thresh` | `f32` | Low detection confidence threshold (for recovery matching stage) |
| `algorithm` | `crate::matching::MatchingAlgorithm` | Algorithm to use for solving the assignment problem |
| `generate_id` | `std::sync::Arc<dyn Fn(Option<&ID>) -> ID + Send + Sync>` | Function to generate new object IDs |

##### Implementations

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Clone**
  - ```rust
    fn clone(self: &Self) -> BytetrackConfig<ID> { /* ... */ }
    ```

- **CloneToUninit**
  - ```rust
    unsafe fn clone_to_uninit(self: &Self, dest: *mut u8) { /* ... */ }
    ```

- **Default**
  - ```rust
    fn default() -> Self { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **ToOwned**
  - ```rust
    fn to_owned(self: &Self) -> T { /* ... */ }
    ```

  - ```rust
    fn clone_into(self: &Self, target: &mut T) { /* ... */ }
    ```

- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

#### Struct `Bytetrack`

Main ByteTrack tracker implementation

ByteTrack is a multi-object tracking algorithm that associates detections with
existing tracks using a two-stage approach:
1. High-confidence detections are matched to existing tracks using IoU
2. Low-confidence detections are used to recover potentially lost tracks

The tracker maintains objects with unique IDs and uses Kalman filtering
for motion prediction between frames.

# Type Parameters
* `ID` - Type used for unique object identifiers (must be hashable and cloneable)

```rust
pub struct Bytetrack<ID> {
    // Some fields omitted
}
```

##### Fields

| Name | Type | Documentation |
|------|------|---------------|
| *private fields* | ... | *Some fields have been omitted* |

##### Implementations

###### Methods

- ```rust
  pub fn new(config: BytetrackConfig<ID>) -> Self { /* ... */ }
  ```
  Create a new ByteTrack tracker with the given configuration

- ```rust
  pub fn track(self: &mut Self, detections: &[&Detection]) -> BytetrackTrackResult<ID> { /* ... */ }
  ```
  Track objects in the current frame

- ```rust
  pub fn iter_objects(self: &Self) -> impl Iterator<Item = (&ID, &Object<ID>)> { /* ... */ }
  ```
  Get a reference to all currently tracked objects

- ```rust
  pub fn get_object_bbox(self: &Self, object_id: &ID) -> Option<BBox> { /* ... */ }
  ```
  Get the current bounding box for an object by ID

- ```rust
  pub fn drain_objects(self: &mut Self) -> Vec<Object<ID>> { /* ... */ }
  ```
  Remove and return all currently tracked objects

- ```rust
  pub fn get_objects(self: &Self) -> Vec<&Object<ID>> { /* ... */ }
  ```
  Get all currently tracked objects

- ```rust
  pub fn last_frame_index(self: &Self) -> usize { /* ... */ }
  ```
  Get last frame index

- ```rust
  pub fn last_id(self: &Self) -> Option<&ID> { /* ... */ }
  ```
  Get last used ID

- ```rust
  pub fn config(self: &Self) -> &BytetrackConfig<ID> { /* ... */ }
  ```
  Get configuration

- ```rust
  pub fn set_config(self: &mut Self, config: &BytetrackConfig<ID>) { /* ... */ }
  ```
  Set configuration

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Default**
  - ```rust
    fn default() -> Self { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

#### Struct `BytetrackTrackResult`

Result of a ByteTrack tracking update operation.

This structure contains the outcomes of processing detections through the ByteTrack algorithm,
categorizing objects into three groups based on their tracking state changes.

# Fields

* `new_objects` - Newly detected objects that have been assigned tracking IDs for the first time.
  Each tuple contains the detection index from the input array and the newly assigned object ID.

* `updated_objects` - Previously tracked objects that have been matched with new detections.
  Each tuple contains the detection index from the input array and the existing object ID.

* `removed_objects` - Objects that were previously tracked but are no longer detected or have
  been determined to be lost. Contains the complete Object instances that were removed.

```rust
pub struct BytetrackTrackResult<ID> {
    pub new_objects: Vec<(usize, ID)>,
    pub updated_objects: Vec<(usize, ID)>,
    pub removed_objects: Vec<crate::object::Object<ID>>,
}
```

##### Fields

| Name | Type | Documentation |
|------|------|---------------|
| `new_objects` | `Vec<(usize, ID)>` | New objects<br>* 0: index of the detection in the input detections array<br>* 1: the created Object Id |
| `updated_objects` | `Vec<(usize, ID)>` | Updated objects<br>* 0: index of the detection in the input detections array<br>* 1: the Object Id |
| `removed_objects` | `Vec<crate::object::Object<ID>>` | Removed objects |

##### Implementations

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

## Module `detection`

```rust
pub mod detection { /* ... */ }
```

### Types

#### Struct `Detection`

Raw detection by the object detector

```rust
pub struct Detection {
    pub bbox: crate::bbox::BBox,
    pub confidence: f32,
}
```

##### Fields

| Name | Type | Documentation |
|------|------|---------------|
| `bbox` | `crate::bbox::BBox` | Bounding box of the detection |
| `confidence` | `f32` | Confidence score of the detection |

##### Implementations

###### Methods

- ```rust
  pub fn new(bbox: BBox, confidence: f32) -> Self { /* ... */ }
  ```
  Create a new detection

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Clone**
  - ```rust
    fn clone(self: &Self) -> Detection { /* ... */ }
    ```

- **CloneToUninit**
  - ```rust
    unsafe fn clone_to_uninit(self: &Self, dest: *mut u8) { /* ... */ }
    ```

- **Debug**
  - ```rust
    fn fmt(self: &Self, f: &mut $crate::fmt::Formatter<''_>) -> $crate::fmt::Result { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **ToOwned**
  - ```rust
    fn to_owned(self: &Self) -> T { /* ... */ }
    ```

  - ```rust
    fn clone_into(self: &Self, target: &mut T) { /* ... */ }
    ```

- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

## Module `kalman`

Kalman filter for bounding box tracking

This module provides a Kalman filter implementation specifically designed for tracking
bounding box parameters: center coordinates (cx, cy), width, height, and their velocities.

```rust
pub mod kalman { /* ... */ }
```

### Types

#### Struct `BBoxKalmanConfig`

Configuration parameters for the bounding box Kalman filter

```rust
pub struct BBoxKalmanConfig {
    pub dt: f32,
    pub process_noise: f32,
    pub measurement_noise: f32,
    pub initial_position_variance: f32,
    pub initial_velocity_variance: f32,
}
```

##### Fields

| Name | Type | Documentation |
|------|------|---------------|
| `dt` | `f32` | Time step for the motion model |
| `process_noise` | `f32` | Process noise strength for position/size uncertainty |
| `measurement_noise` | `f32` | Measurement noise strength |
| `initial_position_variance` | `f32` | Initial position/size uncertainty |
| `initial_velocity_variance` | `f32` | Initial velocity uncertainty   |

##### Implementations

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Clone**
  - ```rust
    fn clone(self: &Self) -> BBoxKalmanConfig { /* ... */ }
    ```

- **CloneToUninit**
  - ```rust
    unsafe fn clone_to_uninit(self: &Self, dest: *mut u8) { /* ... */ }
    ```

- **Debug**
  - ```rust
    fn fmt(self: &Self, f: &mut $crate::fmt::Formatter<''_>) -> $crate::fmt::Result { /* ... */ }
    ```

- **Default**
  - ```rust
    fn default() -> Self { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **ToOwned**
  - ```rust
    fn to_owned(self: &Self) -> T { /* ... */ }
    ```

  - ```rust
    fn clone_into(self: &Self, target: &mut T) { /* ... */ }
    ```

- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

#### Struct `BBoxKalmanFilter`

Kalman filter for tracking bounding box parameters

State vector: [cx, cy, w, h, vx, vy, vw, vh]
- cx, cy: center coordinates
- w, h: width and height
- vx, vy, vw, vh: velocities for each parameter

```rust
pub struct BBoxKalmanFilter {
    // Some fields omitted
}
```

##### Fields

| Name | Type | Documentation |
|------|------|---------------|
| *private fields* | ... | *Some fields have been omitted* |

##### Implementations

###### Methods

- ```rust
  pub fn new(initial_bbox: &BBox, config: BBoxKalmanConfig) -> Result<Self, Box<dyn std::error::Error>> { /* ... */ }
  ```
  Create a new Kalman filter for bounding box tracking

- ```rust
  pub fn predict(self: &mut Self) -> BBox { /* ... */ }
  ```
  Predict the next bounding box state

- ```rust
  pub fn update(self: &mut Self, bbox: &BBox) -> Result<(), Box<dyn std::error::Error>> { /* ... */ }
  ```
  Update the filter with a new bounding box measurement

- ```rust
  pub fn current_bbox(self: &Self) -> BBox { /* ... */ }
  ```
  Get the current estimated bounding box

- ```rust
  pub fn state(self: &Self) -> &[f32] { /* ... */ }
  ```
  Get the current state vector [cx, cy, w, h, vx, vy, vw, vh]

- ```rust
  pub fn covariance(self: &Self) -> &[f32] { /* ... */ }
  ```
  Get the current covariance matrix

- ```rust
  pub fn config(self: &Self) -> &BBoxKalmanConfig { /* ... */ }
  ```
  Get the configuration

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

## Module `matching`

```rust
pub mod matching { /* ... */ }
```

### Types

#### Enum `MatchingAlgorithm`

Assignment algorithms for matching tracks to detections

This enum defines different algorithms that can be used to solve the assignment
problem in multi-object tracking, where we need to match existing tracks to
new detections optimally.

```rust
pub enum MatchingAlgorithm {
    Hungarian {
        scale_factor: f32,
    },
}
```

##### Variants

###### `Hungarian`

Hungarian algorithm (Kuhn-Munkres) for optimal assignment

The scale_factor is used to convert floating-point IoU values to integers
for the integer-based Hungarian algorithm implementation.

Fields:

| Name | Type | Documentation |
|------|------|---------------|
| `scale_factor` | `f32` |  |

##### Implementations

###### Methods

- ```rust
  pub fn solve_assignment(self: &Self, iou_matrix: &[Vec<f32>], iou_min: f32) -> Vec<Option<usize>> { /* ... */ }
  ```
  Solves the assignment problem to match tracks to detections.

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Clone**
  - ```rust
    fn clone(self: &Self) -> MatchingAlgorithm { /* ... */ }
    ```

- **CloneToUninit**
  - ```rust
    unsafe fn clone_to_uninit(self: &Self, dest: *mut u8) { /* ... */ }
    ```

- **Debug**
  - ```rust
    fn fmt(self: &Self, f: &mut $crate::fmt::Formatter<''_>) -> $crate::fmt::Result { /* ... */ }
    ```

- **Default**
  - ```rust
    fn default() -> Self { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **ToOwned**
  - ```rust
    fn to_owned(self: &Self) -> T { /* ... */ }
    ```

  - ```rust
    fn clone_into(self: &Self, target: &mut T) { /* ... */ }
    ```

- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

## Module `object`

```rust
pub mod object { /* ... */ }
```

### Types

#### Struct `Object`

A tracked object with Kalman filter-based prediction

This represents an object being tracked across multiple frames. It maintains
the latest detection information, predicted bounding box from Kalman filtering,
tracking status, and optional point features.

```rust
pub struct Object<ID> {
    pub id: ID,
    pub track: Vec<crate::TrackedDetection>,
    pub status: crate::ObjectStatus,
    // Some fields omitted
}
```

##### Fields

| Name | Type | Documentation |
|------|------|---------------|
| `id` | `ID` |  |
| `track` | `Vec<crate::TrackedDetection>` | Tracked points (for future feature tracking extensions)<br><br>Always contains at least one entry (the initial detection) |
| `status` | `crate::ObjectStatus` | Status of the track (Tracked or Lost with frame count) |
| *private fields* | ... | *Some fields have been omitted* |

##### Implementations

###### Methods

- ```rust
  pub fn from_detection(id: ID, detection: &Detection, detection_index: usize, frame_index: usize) -> Self { /* ... */ }
  ```
  Create a new Object from a detection with default Kalman filter configuration

- ```rust
  pub fn new_with_config(id: ID, detection: &Detection, detection_index: usize, frame_index: usize, config: BBoxKalmanConfig) -> Self { /* ... */ }
  ```
  Create a new Object with custom Kalman filter configuration

- ```rust
  pub fn predict(self: &mut Self) -> BBox { /* ... */ }
  ```
  Predict the next bounding box position using Kalman filter

- ```rust
  pub fn update(self: &mut Self, detection: &Detection, detection_index: usize, frame_index: usize) -> Result<(), Box<dyn std::error::Error>> { /* ... */ }
  ```
  Update the object with a new matching detection

- ```rust
  pub fn current_bbox(self: &Self) -> BBox { /* ... */ }
  ```
  Get the current bounding box (from the latest tracked detection or Kalman filter)

- ```rust
  pub fn last_tracked_detection(self: &Self) -> &TrackedDetection { /* ... */ }
  ```
  Get the latest [TrackedDetection]

- ```rust
  pub fn last_seen_index(self: &Self) -> Option<usize> { /* ... */ }
  ```
  Get the index of the last seen [Detection]

- ```rust
  pub fn last_seen_detection(self: &Self) -> Detection { /* ... */ }
  ```
  Get the last seen [Detection]

- ```rust
  pub fn last_seen_tracked_detection(self: &Self) -> &TrackedDetection { /* ... */ }
  ```
  Get the [TrackedDetection] where the detection was last seen

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Clone**
  - ```rust
    fn clone(self: &Self) -> Self { /* ... */ }
    ```

- **CloneToUninit**
  - ```rust
    unsafe fn clone_to_uninit(self: &Self, dest: *mut u8) { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **ToOwned**
  - ```rust
    fn to_owned(self: &Self) -> T { /* ... */ }
    ```

  - ```rust
    fn clone_into(self: &Self, target: &mut T) { /* ... */ }
    ```

- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

## Module `object_status`

```rust
pub mod object_status { /* ... */ }
```

### Types

#### Enum `ObjectStatus`

Status of a tracked object

Objects can be either actively tracked (when recently matched to detections)
or temporarily lost (when no matching detection was found for some frames).

```rust
pub enum ObjectStatus {
    Tracked,
    Lost {
        frames: u32,
    },
}
```

##### Variants

###### `Tracked`

Object is actively being tracked

###### `Lost`

Object is temporarily lost for the specified number of frames

Fields:

| Name | Type | Documentation |
|------|------|---------------|
| `frames` | `u32` |  |

##### Implementations

###### Methods

- ```rust
  pub fn incrment_lost_frames(self: &mut Self) { /* ... */ }
  ```
  Increment the number of lost frames or transition from Tracked to Lost

- ```rust
  pub fn get_lost_frames(self: &Self) -> u32 { /* ... */ }
  ```
  Get the number of frames this object has been lost

- ```rust
  pub fn set_tracked(self: &mut Self) { /* ... */ }
  ```
  Set status to Tracked

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Clone**
  - ```rust
    fn clone(self: &Self) -> ObjectStatus { /* ... */ }
    ```

- **CloneToUninit**
  - ```rust
    unsafe fn clone_to_uninit(self: &Self, dest: *mut u8) { /* ... */ }
    ```

- **Debug**
  - ```rust
    fn fmt(self: &Self, f: &mut $crate::fmt::Formatter<''_>) -> $crate::fmt::Result { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **ToOwned**
  - ```rust
    fn to_owned(self: &Self) -> T { /* ... */ }
    ```

  - ```rust
    fn clone_into(self: &Self, target: &mut T) { /* ... */ }
    ```

- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

## Module `tracked_detection`

```rust
pub mod tracked_detection { /* ... */ }
```

### Types

#### Struct `TrackedDetection`

```rust
pub struct TrackedDetection {
    pub detection: Option<crate::Detection>,
    pub detection_index: Option<usize>,
    pub kalman_bbox: crate::BBox,
    pub frame_index: usize,
}
```

##### Fields

| Name | Type | Documentation |
|------|------|---------------|
| `detection` | `Option<crate::Detection>` |  |
| `detection_index` | `Option<usize>` |  |
| `kalman_bbox` | `crate::BBox` |  |
| `frame_index` | `usize` |  |

##### Implementations

###### Methods

- ```rust
  pub fn new(detection: Option<&Detection>, detection_index: Option<usize>, kalman_bbox: BBox, frame_index: usize) -> Self { /* ... */ }
  ```

###### Trait Implementations

- **Any**
  - ```rust
    fn type_id(self: &Self) -> TypeId { /* ... */ }
    ```

- **Borrow**
  - ```rust
    fn borrow(self: &Self) -> &T { /* ... */ }
    ```

- **BorrowMut**
  - ```rust
    fn borrow_mut(self: &mut Self) -> &mut T { /* ... */ }
    ```

- **Clone**
  - ```rust
    fn clone(self: &Self) -> TrackedDetection { /* ... */ }
    ```

- **CloneToUninit**
  - ```rust
    unsafe fn clone_to_uninit(self: &Self, dest: *mut u8) { /* ... */ }
    ```

- **Freeze**
- **From**
  - ```rust
    fn from(t: T) -> T { /* ... */ }
    ```
    Returns the argument unchanged.

- **Into**
  - ```rust
    fn into(self: Self) -> U { /* ... */ }
    ```
    Calls `U::from(self)`.

- **RefUnwindSafe**
- **Same**
- **Send**
- **SupersetOf**
  - ```rust
    fn to_subset(self: &Self) -> Option<SS> { /* ... */ }
    ```

  - ```rust
    fn is_in_subset(self: &Self) -> bool { /* ... */ }
    ```

  - ```rust
    fn to_subset_unchecked(self: &Self) -> SS { /* ... */ }
    ```

  - ```rust
    fn from_subset(element: &SS) -> SP { /* ... */ }
    ```

- **Sync**
- **ToOwned**
  - ```rust
    fn to_owned(self: &Self) -> T { /* ... */ }
    ```

  - ```rust
    fn clone_into(self: &Self, target: &mut T) { /* ... */ }
    ```

- **TryFrom**
  - ```rust
    fn try_from(value: U) -> Result<T, <T as TryFrom<U>>::Error> { /* ... */ }
    ```

- **TryInto**
  - ```rust
    fn try_into(self: Self) -> Result<U, <U as TryFrom<T>>::Error> { /* ... */ }
    ```

- **Unpin**
- **UnwindSafe**
- **VZip**
  - ```rust
    fn vzip(self: Self) -> V { /* ... */ }
    ```

## Macros

### Macro `bbox_xyxy`

**Attributes:**

- `#[macro_export]`

Convenience macro for creating a bounding box from (x1, y1, x2, y2) coordinates

# Example
```rust
use bytetrack::bbox_xyxy;
let bbox = bbox_xyxy!(10.0, 20.0, 30.0, 40.0);
```

```rust
pub macro_rules! bbox_xyxy {
    /* macro_rules! bbox_xyxy {
    ($x1:expr, $y1:expr, $x2:expr, $y2:expr) => { ... };
} */
}
```

### Macro `bbox`

**Attributes:**

- `#[macro_export]`

Convenience macro for creating a bounding box from (x, y, width, height)

# Example
```rust
use bytetrack::bbox;
let bbox = bbox!(10.0, 20.0, 30.0, 40.0);
```

```rust
pub macro_rules! bbox {
    /* macro_rules! bbox {
    ($x:expr, $y:expr, $w:expr, $h:expr) => { ... };
} */
}
```

### Macro `det`

**Attributes:**

- `#[macro_export]`

```rust
pub macro_rules! det {
    /* macro_rules! det {
    ($point:expr, $conf:expr) => { ... };
} */
}
```

## Re-exports

### Re-export `bbox::*`

```rust
pub use bbox::*;
```

### Re-export `bytetrack::*`

```rust
pub use bytetrack::*;
```

### Re-export `detection::*`

```rust
pub use detection::*;
```

### Re-export `kalman::*`

```rust
pub use kalman::*;
```

### Re-export `matching::*`

```rust
pub use matching::*;
```

### Re-export `object::*`

```rust
pub use object::*;
```

### Re-export `object_status::*`

```rust
pub use object_status::*;
```

### Re-export `tracked_detection::*`

```rust
pub use tracked_detection::*;
```

