# Texture Track Line Extraction

This package extracts matched `left_line`, `center_line`, and `right_line` coordinates from the provided textured track image.

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

## Output columns

- `seq`
- `left_x_px`, `left_y_px`
- `center_x_px`, `center_y_px`
- `right_x_px`, `right_y_px`
- `left_x_mm`, `left_y_mm`
- `center_x_mm`, `center_y_mm`
- `right_x_mm`, `right_y_mm`
- `lane_width_px`, `lane_width_mm`

## Validation

The included result was generated with `--samples 500`. Therefore the CSV has 500 rows and `seq` runs from 0 to 499 without gaps.
