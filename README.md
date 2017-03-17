# Stitch
Fixing vertical video bars.

## How it works
- Each frame find the sift descriptors
- Match the descriptors to infer depth and movement
- Stich adjacent frames until image is of proper size
- Output video resolution should be whatever you specify

## Issues / TODO
- do not load all frames at once
  - JVM with 4GB heap space was not enough to load ~750 frames
- do not preserve aspect ratio, simply use the dimensions provided by the user
  - crop if necessary
