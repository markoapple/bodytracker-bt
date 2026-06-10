# Manual plank floor calibration

Automatic floorboard detection has been removed. Floor geometry now comes from one user-drawn plank outline.

## UI flow

1. Refresh the Camera A preview.
2. Click **Draw one plank**.
3. Drag along the first long edge of one visible plank.
4. Drag along the opposite long edge of the same plank.
5. Drag one short end cap across those long edges.
6. Optional but recommended: drag the opposite short end cap.
7. Enter the real plank width and length if you want metric projective geometry.
8. Click **Apply drawn plank** if it has not already applied, then save the config.

The backend receives manual endpoints and preview image size. Real plank size is optional. If width/length are not provided, the result is an orientation/geometry hint, not metric scale truth.

## What the result means

The manual calibration extracts:

- long-edge directions and lengths
- end-cap directions and lengths
- end-cap corner intersections
- plank pixel width at each end cap
- approximate plank pixel length between end caps
- scalar-spacing diagnostics for the old one-axis floor assist path
- optional metric width and length if the user provides real dimensions
- projective perspective evidence: cap width ratio, long-edge length ratio, and vanishing points when the drawn edge pairs converge
- projective homography when four corners plus real width and length are available
- camera height when the user provided a valid camera height

Manual plank lines do **not** extract lens/radial distortion. The UI sends endpoint lines, not sampled curves. Distortion coefficients need sampled seam curvature, so the saved manual calibration explicitly marks lens distortion unavailable instead of inventing coefficients. Straight endpoint geometry still extracts projective perspective distortion: the trapezoid, vanishing points, and four-corner homography are the usable “camera bend” evidence from one plank.

Saved calibration is marked with:

- `source: manual_plank_outline`
- `floor_type: manual_plank`
- `homography_valid: true` only for a full four-corner plank with real width and length
- `homography_reason: manual_plank_quad_projective_homography_from_four_corners` when projective geometry is solved
- `family_a.vanishing_point_valid` / `family_b.vanishing_point_valid` when the drawn edge pairs give finite projective vanishing points
- `manual_plank.projective_perspective_observed` when the outline contains perspective evidence
- `distortion.reason: manual_plank_endpoint_lines_do_not_estimate_lens_distortion_projective_perspective_is_homography`


## Important runtime limits

Manual plank geometry is saved in the image coordinate system it was calibrated in. The saved calibration records `image_width` / `image_height` so runtime can compare that source coordinate space with the current monocular processing space.

Do not throw the geometry away just because the pixel dimensions changed. If the resize, crop, letterbox, or processing transform from saved calibration space to current keypoint space is known, remap the endpoints, corners, scalar spacing, reference y, and homography inputs through that transform and keep using the evidence with honest status. Dimension mismatch is not automatically invalid; it is a coordinate-mapping problem. Solve the mapping when the pipeline knows it.

Only suppress the raw image-space floor references when the coordinate mapping is genuinely unknown, non-finite, contradictory, or unconfigured. In that case, report that the calibration cannot be mapped into the current processing image and ask for recalibration. Do not label known-resize geometry as dead evidence.

A single short end cap is saved as plank outline evidence only. It is not treated as a second repeated line family. The fourth, opposite end cap is required before the calibration claims a two-axis/projective plank quad.

After reload, saved metric width/length are displayed from the backend calibration. Changing those fields does not rewrite an existing homography unless the manual outline is still present and can be recomputed through the backend.
