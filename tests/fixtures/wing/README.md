# Wing fixture: naca2412_wing_r150_t120_s120_w1.5.stl

## Description
A tapered AND twisted NACA 2412 wing panel -- the first fixture in this
repository that varies chord and rotation at the same time, and the first
above 60 mm chord. Chord runs 150 mm at the root (Z=0) to 120 mm at the tip
(Z=120 mm), linear in Z. The section simultaneously rotates about the
quarter-chord point by 1.5 degrees of washout, linear in Z.

Because chord varies, the quarter-chord pivot traces a SWEPT line down the
span rather than a straight one. That is intended: it is what a real
tapered wing rotating about its quarter-chord does.

## Why it exists
D022 established that taper and twist are one mechanism with two sources of
far-surface slope, and that correcting for global rotation alone is short in
magnitude because local slope subsumes rotation. No single-mechanism fixture
can exercise that. This one carries both at a realistic chord.

## Built geometry, read back (not asserted)
- chord-line angle at root: 0.0000 deg
- chord-line angle at tip:  1.5000 deg (positive washout tilts nose-down)
- taper ratio: 0.8000
- triangles: 19512
- sha256: `14eef2a443c2d5fab977dc36cd4a070ae101ec6ef442e2bff055dd89fd60a99f`

## Origin convention
Same root-bounding-box-centring convention as the taper, sweep and twist
fixtures (shared `build_loft_stations`). Z is spanwise, root at Z=0.

## Section thickness
| Z (mm) | Chord (mm) | Local max thickness (mm) |
|---|---|---|
| 0 | 150.0000 | 17.9965 |
| 30 | 142.5000 | 17.0966 |
| 60 | 135.0000 | 16.1968 |
| 90 | 127.5000 | 15.2970 |
| 120 | 120.0000 | 14.3972 |
