# Models

Project-local model checkpoints used by AAC perception nodes.

- `yolo26n-sem.pt`: Ultralytics semantic segmentation checkpoint used by `road_seg_node`.
- `yolo26n-seg.pt`: Ultralytics instance segmentation checkpoint for `road_seg_node` segment-mode smoke tests.

YOLOP checkpoints are kept under `perception/YOLOP/weights/` because they belong
to the vendored YOLOP repository layout.
