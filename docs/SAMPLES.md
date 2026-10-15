# Public samples and evidence

## Actual hardware media

Source: https://x.com/OsoneHiroyuki/status/2110614582570164644
Author: Hiroyuki Osone (@OsoneHiroyuki), EmplifAI.
The original public video is approximately 286 seconds long.

| File | Source position | Interpretation |
| --- | --- | --- |
| `assets/hardware-demo.jpg` | X video poster | Robot and bottle before the shown sequence |
| `assets/demo-060s.jpg` | 60 seconds | Bottle held upright above the tabletop |
| `assets/demo-090s.jpg` | 90 seconds | Bottle at tabletop, hand still near it |
| `assets/demo-120s.jpg` | 120 seconds | Hand withdrawn, bottle visible on table |

Frames were extracted from the public 480×852 MP4 without inventing or replacing
scene content. A frame is not proof of calibrated pose, forces, closed-loop
reasoning, intervention count, or repeatability. The video and author's report
are evidence of the lab demonstration, not a success-rate study.

`examples/public-demo.json` records this provenance in machine-readable form.
It deliberately contains no fabricated sensor or timing measurements.

## UI and illustration

`assets/console-desktop.jpg` is an actual screenshot of the local English console
with no API key or active hardware connection. `../web/sample.svg` is an original
illustration. The UI labels it as such, and informs the model that it is not a
sensor frame. Upload a public demo JPEG to inspect real imagery instead.

## What is not released

No synchronized RGB-D/joint recording, calibrated force trace, private lab image,
API response archive, or dataset of repeated trials is included in this release.
Runtime recordings stay ignored under `captures/`, `records/` and `logs/`.
An evidence dataset should document consent, timestamps, robot/configuration,
interventions, outcome definitions and failure cases before publication.
