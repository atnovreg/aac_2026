# Track line extraction package

This package contains the input drawing image, a preview image, a wide-format CSV, and a Python program that extracts matched lane coordinates.

## Definitions

- `right_line`: outer red lane boundary
- `left_line`: inner red lane boundary
- `center_line`: midpoint between `left_line` and `right_line`
- `seq`: matched sample index. The same `seq` row contains corresponding left / center / right points.

## Files

- `input_track_image.png`: original input image
- `extracted_lines_preview.png`: preview of extracted left / center / right lines
- `track_lines_wide.csv`: extracted wide-format CSV
- `extract_track_lines_wide.py`: Python extraction program
- `requirements.txt`: minimal Python dependencies

## Run

```bash
pip install -r requirements.txt
python extract_track_lines_wide.py \
  --input input_track_image.png \
  --output-csv track_lines_wide.csv \
  --preview extracted_lines_preview.png \
  --samples 500 \
  --scale-width-mm 16000 \
  --scale-height-mm 11000
```

## CSV columns

```text
seq,
left_x_px,left_y_px,left_x_mm,left_y_mm,
center_x_px,center_y_px,center_x_mm,center_y_mm,
right_x_px,right_y_px,right_x_mm,right_y_mm
```

`*_px` values are image pixel coordinates. `*_mm` values are approximate drawing coordinates scaled from the image size using the provided width/height options.
For exact CAD coordinates, extract from the original DXF/DWG layers instead of the raster PNG.
