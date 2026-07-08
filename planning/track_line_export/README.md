# Texture Track Line Extraction

This package extracts matched `left_line`, `center_line`, and `right_line` coordinates from the provided textured track image.
It also includes a separate parking-lot marking extractor for future parking
mission planning.

## Convention

- `left_line` = inner road boundary
- `right_line` = outer road boundary
- `center_line` = point-wise midpoint of left/right
- CSV uses **wide format**, so the same `seq` row contains left/center/right coordinates sampled at the same index.

## Run

```bash
pip install -r requirements.txt

python extract_texture_track_lines_wide.py \
  --input input_track_texture.png \
  --output-csv track_texture_lines_wide.csv \
  --preview track_texture_lines_preview.png \
  --samples 500 \
  --scale-width-mm 16000 \
  --scale-height-mm 11000
```

Parking-lot line extraction is kept separate from the driving centerline CSV:

```bash
python extract_texture_parking_lines.py \
  --input input_track_texture.png \
  --output-csv parking_lot_lines.csv \
  --preview parking_lot_lines_preview.png
```

## Output columns

- `seq`
- `left_x_px`, `left_y_px`
- `center_x_px`, `center_y_px`
- `right_x_px`, `right_y_px`
- `left_x_mm`, `left_y_mm`
- `center_x_mm`, `center_y_mm`
- `right_x_mm`, `right_y_mm`
- `lane_width_px`, `lane_width_mm`

`parking_lot_lines.csv` uses one row per extracted marking segment:

- `line_id`, `line_type`, `line_role`, `orientation`
- `x1_px`, `y1_px`, `x2_px`, `y2_px`
- `x1_mm`, `y1_mm`, `x2_mm`, `y2_mm`
- `x1_m`, `y1_m`, `x2_m`, `y2_m`
- `center_x_*`, `center_y_*`, `length_*`, `angle_deg`, `evidence_count`

The world-coordinate conversion matches `track_path_publisher` defaults:
`map_swap_xy=true`, `map_flip_x=true`, `map_flip_y=true`, map bounds
`x=[-5.5, 5.5]`, `y=[-8.0, 8.0]`.

## Validation

The included result was generated with `--samples 500`. Therefore the CSV has 500 rows and `seq` runs from 0 to 499 without gaps.

The parking-lot result is summarized in `parking_lot_lines_validation_summary.txt`.
