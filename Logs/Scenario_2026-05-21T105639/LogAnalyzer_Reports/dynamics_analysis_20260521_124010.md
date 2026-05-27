# Dynamics Analysis - Scenario_2026-05-21T105639

- Saved at: `2026-05-21T12:40:10+09:00`
- Scenario path: `C:\Users\LAHMUMT_2\Desktop\DSS_KU\Logs\Scenario_2026-05-21T105639`
- 0401 files: `6`
- 0401 samples: `21726`
- Quality: `ok`

## Cohort Summary

| Metric | Value |
| --- | --- |
| Usable UAV | 3 / 3 |
| Speed p50 | 44.86 m/s |
| Turn radius p50 | 538.8 m |
| Yaw rate p95 | 6.522 dps |
| Bank proxy p95 | 28.17 deg |
| Command latency p50 | 0.273 s |

## Planning / Runtime Fit

| Metric | Value |
| --- | --- |
| Median speed | 44.86 m/s |
| Median turn radius | 538.8 m |
| Current reference radius | 503.5 m |
| Planner turn radius scale | 1.0701x |
| Nominal turn radius | 538.8 m |
| Conservative turn radius | 540.2 m |
| Aggressive turn radius | 534.4 m |
| Lookahead | 242.4 m |
| Freeze yaw distance | 75.4 m |

## SIM-Compatible Dynamics Profile

```json
{
  "banked_turn_enabled": true,
  "bank_yaw_rate_blend": 0.85,
  "reference_turn_radius_scale": 1.0701,
  "max_yaw_rate_dps": 7.967,
  "max_roll_rate_dps": 9.914,
  "turn_bank_limit_deg": 31.697,
  "roll_limit_deg": 36.697,
  "turn_roll_gain": 2.2,
  "use_reference_turn_radius": true
}
```

## Aircraft Breakdown

| AC | UAV | Samples | Duration | V50 | R50 | R75 | Yaw95 | Bank95 | Roll |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| LAH1 | no | 3621 | 724.0 s | 0.0 m/s | - | - | 0.00 dps | - | proxy |
| LAH2 | no | 3621 | 724.0 s | 102.7 m/s | - | - | 0.00 dps | - | proxy |
| LAH3 | no | 3621 | 724.0 s | 102.2 m/s | - | - | 0.00 dps | - | proxy |
| UAV1 | yes | 3621 | 724.0 s | 44.9 m/s | 539 m | 641 m | 6.98 dps | 28.1 deg | proxy |
| UAV2 | yes | 3621 | 724.0 s | 44.9 m/s | 530 m | 622 m | 6.52 dps | 28.4 deg | proxy |
| UAV3 | yes | 3621 | 724.0 s | 44.9 m/s | 542 m | 632 m | 6.35 dps | 28.2 deg | proxy |

## Mission Phase Sensitivity

| AC | Phase | Samples | V50 | R50 | Yaw95 | Bank95 |
| --- | --- | --- | --- | --- | --- | --- |
| LAH1 | unknown | 3620 | 0.0 m/s | - | 0.00 dps | - |
| LAH2 | unknown | 3620 | 102.7 m/s | - | 0.00 dps | - |
| LAH3 | unknown | 3620 | 102.2 m/s | - | 0.00 dps | - |
| UAV1 | target-track | 2770 | 44.9 m/s | 522 m | 6.73 dps | 29.0 deg |
| UAV2 | target-track | 2503 | 44.9 m/s | 514 m | 6.99 dps | 29.8 deg |
| UAV3 | target-track | 2455 | 44.9 m/s | 527 m | 6.88 dps | 29.7 deg |
| UAV3 | imaging | 1165 | 38.8 m/s | 2303 m | 3.13 dps | 18.0 deg |
| UAV2 | imaging | 1117 | 38.8 m/s | 3254 m | 2.87 dps | 18.1 deg |
| UAV1 | imaging | 841 | 38.8 m/s | 2232 m | 7.16 dps | 27.1 deg |
| UAV1 | imaging:hold | 9 | 39.9 m/s | 446 m | 5.53 dps | 21.5 deg |

## 0602 Command Response

- Matched: `42 / 42`
- Latency p50: `0.273 s`
- Latency p95: `0.618 s`

| AC | Command | State | Latency | Expected |
| --- | --- | --- | --- | --- |
| UAV2 | flightMode | matched | 0.618 s | {"flightMode": 7, "startWaypointId": 58} |
| UAV3 | flightMode | matched | 0.612 s | {"flightMode": 7, "startWaypointId": 65} |
| UAV1 | flightMode | matched | 0.610 s | {"flightMode": 7, "startWaypointId": 50} |
| UAV2 | filming | matched | 0.507 s | {"sensorMode": 1, "fov": 11.7} |
| UAV3 | filming | matched | 0.494 s | {"sensorMode": 1, "fov": 11.7} |
| UAV1 | filming | matched | 0.492 s | {"sensorMode": 1, "fov": 11.7} |
| UAV3 | flightMode | matched | 0.015 s | {"flightMode": 7, "startWaypointId": 101} |
| UAV1 | flightMode | matched | 0.172 s | {"flightMode": 7, "startWaypointId": 93} |
| UAV2 | flightMode | matched | 0.140 s | {"flightMode": 7, "startWaypointId": 97} |
| UAV3 | filming | matched | 0.110 s | {"sensorMode": 1, "fov": 7.7} |
| UAV1 | filming | matched | 0.063 s | {"sensorMode": 1, "fov": 7.7} |
| UAV2 | filming | matched | 0.032 s | {"sensorMode": 1, "fov": 7.7} |
| UAV1 | filming | matched | 0.045 s | {"sensorMode": 1, "fov": 0.61} |
| UAV1 | filming | matched | 0.124 s | {"sensorMode": 1, "fov": 7.7} |
| UAV1 | filming | matched | 0.124 s | {"sensorMode": 1, "fov": 0.61} |
| UAV1 | filming | matched | 0.151 s | {"sensorMode": 1, "fov": 7.7} |
| UAV1 | filming | matched | 0.186 s | {"sensorMode": 2, "fov": 7.7} |
| UAV2 | filming | matched | 0.142 s | {"sensorMode": 2, "fov": 7.7} |
| UAV2 | filming | matched | 0.212 s | {"sensorMode": 1, "fov": 1.4847131} |
| UAV1 | filming | matched | 0.407 s | {"sensorMode": 1, "fov": 2.4} |
| UAV2 | filming | matched | 0.592 s | {"sensorMode": 2, "fov": 7.7} |
| UAV1 | filming | matched | 0.476 s | {"sensorMode": 2, "fov": 7.7} |
| UAV1 | filming | matched | 0.250 s | {"sensorMode": 1, "fov": 2.3328688} |
| UAV1 | filming | matched | 0.023 s | {"sensorMode": 3, "fov": 1.8} |
| UAV1 | flightMode | matched | 0.402 s | {"flightMode": 9, "targetId": 74} |
| UAV2 | filming | matched | 0.313 s | {"sensorMode": 1, "fov": 1.6779348} |
| UAV2 | filming | matched | 0.559 s | {"sensorMode": 2, "fov": 7.7} |
| UAV3 | filming | matched | 0.158 s | {"sensorMode": 2, "fov": 7.7} |
| UAV2 | filming | matched | 0.233 s | {"sensorMode": 1, "fov": 1.7440007} |
| UAV2 | filming | matched | 0.481 s | {"sensorMode": 2, "fov": 7.7} |
| UAV2 | filming | matched | 0.540 s | {"sensorMode": 1, "fov": 1.3777082} |
| UAV2 | filming | matched | 0.329 s | {"sensorMode": 2, "fov": 7.7} |
| UAV3 | filming | matched | 0.201 s | {"sensorMode": 1, "fov": 1.6498593} |
| UAV3 | filming | matched | 0.669 s | {"sensorMode": 2, "fov": 7.7} |
| UAV3 | filming | matched | 0.469 s | {"sensorMode": 1, "fov": 1.912693} |
| UAV2 | filming | matched | 0.296 s | {"sensorMode": 1, "fov": 1.3383048} |
| UAV3 | filming | matched | 0.719 s | {"sensorMode": 2, "fov": 7.7} |
| UAV2 | filming | matched | 0.058 s | {"sensorMode": 3, "fov": 1.8} |
| UAV2 | flightMode | matched | 0.418 s | {"flightMode": 9, "targetId": 75} |
| UAV3 | filming | matched | 0.544 s | {"sensorMode": 1, "fov": 1.616853} |

## Aggressive Turn Events

| AC | Phase | Duration | Yaw Max | Radius Min | Bank Max | Turn Angle |
| --- | --- | --- | --- | --- | --- | --- |
| LAH2 | unknown | 1.80 s | 32.90 dps | - | - | 34 deg |
| LAH3 | unknown | 1.80 s | 27.56 dps | - | - | 31 deg |
| UAV1 | imaging | 6.00 s | 15.89 dps | 141 m | 47.8 deg | 48 deg |
| UAV3 | imaging | 2.81 s | 14.97 dps | 145 m | 45.3 deg | 21 deg |
| UAV1 | imaging | 4.61 s | 13.95 dps | 164 m | 44.7 deg | 32 deg |
| UAV1 | imaging | 6.20 s | 13.94 dps | 160 m | 44.0 deg | 44 deg |
| UAV1 | imaging | 0.41 s | 13.71 dps | 163 m | 43.5 deg | 6 deg |
| UAV2 | target-track | 4.19 s | 13.46 dps | 190 m | 46.9 deg | 29 deg |
| UAV2 | target-track | 1.81 s | 13.10 dps | 196 m | 46.3 deg | 13 deg |
| UAV1 | target-track | 5.00 s | 12.90 dps | 199 m | 45.8 deg | 32 deg |
| UAV1 | imaging | 1.20 s | 12.79 dps | 179 m | 42.3 deg | 11 deg |
| UAV2 | target-track | 4.59 s | 12.73 dps | 201 m | 45.4 deg | 30 deg |
| UAV2 | target-track | 0.60 s | 12.53 dps | 204 m | 44.8 deg | 6 deg |
| UAV2 | target-track | 4.60 s | 12.52 dps | 205 m | 45.0 deg | 31 deg |
| UAV2 | target-track | 3.40 s | 12.37 dps | 208 m | 44.7 deg | 24 deg |
| UAV3 | target-track | 1.01 s | 11.91 dps | 215 m | 43.5 deg | 8 deg |
| UAV1 | target-track | 0.40 s | 11.90 dps | 216 m | 43.5 deg | 4 deg |
| UAV1 | target-track | 4.81 s | 11.83 dps | 217 m | 43.4 deg | 30 deg |
| UAV1 | target-track | 0.99 s | 11.68 dps | 220 m | 43.0 deg | 8 deg |
| UAV1 | target-track | 5.00 s | 11.61 dps | 222 m | 42.8 deg | 32 deg |
| UAV3 | target-track | 3.60 s | 11.57 dps | 221 m | 42.6 deg | 24 deg |
| UAV3 | target-track | 0.99 s | 11.44 dps | 225 m | 42.4 deg | 8 deg |
| UAV2 | target-track | 2.59 s | 11.39 dps | 226 m | 42.3 deg | 18 deg |
| UAV2 | target-track | 3.20 s | 11.31 dps | 227 m | 42.1 deg | 20 deg |
| UAV3 | target-track | 4.00 s | 11.20 dps | 229 m | 41.8 deg | 25 deg |
| UAV3 | target-track | 0.40 s | 11.19 dps | 230 m | 41.8 deg | 4 deg |
| UAV3 | target-track | 2.20 s | 11.12 dps | 231 m | 41.5 deg | 15 deg |
| UAV2 | target-track | 1.00 s | 11.10 dps | 232 m | 41.5 deg | 8 deg |
| UAV2 | target-track | 2.79 s | 11.07 dps | 232 m | 41.5 deg | 18 deg |
| UAV2 | target-track | 0.59 s | 11.04 dps | 233 m | 41.4 deg | 6 deg |

## All-Log Signal Scan

- Files scanned: `261`
- Sampled bytes: `39363923`
- Roll fields: `present`

| Signal | Count |
| --- | --- |
| flight_mode | 13662 |
| heading | 21299 |
| pitch | 4598 |
| replan | 321 |
| roll_or_bank | 4598 |
| sensor | 18030 |
| target | 17389 |
| turn_radius | 109 |
| waypoint | 11362 |
| yaw | 4598 |
