# RMCS traditional detector port

Source: the user's local `rmcs_auto_aim_v2-main`, recorded in SOURCE.json.
RMCS sources retain the Alliance Algorithm Team MIT license in LICENSE.

`rune.cpp/.hpp` are copied from upstream. Rune geometry, aimpoints and CampColor now live in auto_buff/buff_types.hpp.
Color extraction lives in ../process.hpp/.cpp; common mathematical helpers
remain in rm_utils/algorithm. SOURCE.json records their current destinations.
Color thresholds, the 3x3 dilation, camera/distance
size gates, contour filters, R skeleton scoring, contour moments, polar bins,
fourth harmonic, PVD classification and radial endpoint extraction are unchanged.

Detector edits relocate shared includes and replace its ximgproc include/call with
`compat/thinning.hpp` / `cv::rmcs_compat::thinning`. The Windows OpenCV SDK
has no contrib module, so compat/thinning.cpp carries OpenCV contrib 4.12.0's
official implementation. Its include and namespace are isolated; its final
`hasNonZero(diff)` check becomes `countNonZero(diff) != 0` for OpenCV 4.6
compatibility. The original source URL/hash and Apache 2.0 license are retained.
Tests compare the used Zhang-Suen implementation against an independent
neighbor/transition definition, including border pixels.

`../traditional.cpp` is the standalone adapter:

- Read validated camera parameters and require the input resolution to match.
- Expose the five existing RMCS distance/perspective/score settings.
- Run the red or blue color selected by configuration/control, represented internally by CampColor.
- Map every bullseye center, PVD and state without temporal confirmation.
- Preserve all current R candidates; the tracker handles color grouping and
  submits all same-color candidates to the original PnP/association logic.
- Keep the original four-arm endpoints on pending targets; reject off-image
  endpoints as initialization features. Upstream active-ring radius placeholders
  are used for display radius only, never published as physical arm endpoints.
- Optionally return the original dilated color mask for the selected color.

No image template, candidate history, fitted-ellipse center replacement or
additional pixel-quality gates are part of the traditional branch anymore.
The upstream header's reversed active/inactive PVD comments have been corrected;
the assignment is unchanged: PVD below active_threshold is
`active=true` (single ring), otherwise `active=false` (pending four-arm target).

This is a source-faithful detection baseline, not a claim that every supplied
video reproduces the upstream demonstration. Input exposure, compression,
overlays, camera parameters and the exact upstream version still matter.
