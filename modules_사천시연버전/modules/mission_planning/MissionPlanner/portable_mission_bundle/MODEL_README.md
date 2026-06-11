# Bundled Model

This folder contains a copied inference model at:

- `models/latest_model.zip`

Copied from the source project:

- `training/runs/ppo_uav_20251110-001841/ppo_uav_final.zip`

Copied configuration:

- `models/model_config.json`

Default environment values derived from the copied config:

- `altitude_m`: `700.0`
- `hex_step`: `25`
- `max_steps`: `3000`
- `max_goals`: `20`
- `crash_penalty`: `-5.0`
- `step_penalty`: `-0.1`
- `success_reward`: `10.0`
- `goal_reward`: `5.0`
- `progress_scale`: `0.2`
- `proximity_penalty`: `0.02`

Action space:

- `0`: Turn Left
- `1`: Go Straight
- `2`: Turn Right

Observation layout expected by the model:

1. normalized current row
2. normalized current col
3. heading index
4. normalized target row
5. normalized target col
6. remaining-goal ratio
7. delta row
8. delta col
9. six near-ring safety values
10. twelve far-ring safety values

This bundle is intended for inference only.
Training, dashboards, and analysis tools were intentionally left out.
