const state = {
  mission: null,
  replan: null,
  recommendedReplan: null,
  paths: {},
};

const $ = (selector) => document.querySelector(selector);

const missionSections = [
  {
    title: "운용 / FOV",
    fields: [
      { path: "values.enhanced_auto_fov_from_db", label: "FOV DB 자동 적용", type: "bool" },
      {
        path: "values.capture_mission_param_mode",
        label: "촬영 임무 파라미터 모드",
        type: "select",
        options: [
          ["공통값 사용", "common"],
          ["수동값 사용", "manual"],
        ],
      },
      { path: "values.camera_adjust_enabled", label: "카메라 보정 사용", type: "bool" },
      { path: "values.camera_adjust_percent", label: "카메라 보정 비율(%)", type: "number", min: -90, max: 90, step: 1 },
      { path: "values.global_manual_fov_deg", label: "수동 FOV(deg)", type: "number", min: 1.2, max: 120, step: 0.1 },
      { path: "values.default_sweep_separation_m", label: "기본 Sweep 간격(m)", type: "number", min: 10, step: 10 },
      { path: "values.fov_db_sep_safety_factor", label: "FOV DB SEP 보수 배수", type: "number", min: 1, max: 5, step: 0.05 },
      { path: "values.fov_db_path", label: "FOV DB 경로", type: "text" },
      { path: "values.db_fov_weight", label: "DB FOV 반영 배수", type: "number", min: 0.1, max: 5, step: 0.05 },
      { path: "values.fov_db_smaller_fov_steps", label: "Line DB lower steps", type: "int", min: 0, max: 20, step: 1 },
      { path: "values.area_fov_db_smaller_fov_steps", label: "Area DB lower steps", type: "int", min: 0, max: 20, step: 1 },
    ],
  },
  {
    title: "비행 / 경로",
    fields: [
      { path: "values.uav_wp_interval_m", label: "UAV WP 간격(m)", type: "number", min: 100, step: 100 },
      { path: "values.lah_wp_interval_m", label: "LAH WP 간격(m)", type: "number", min: 100, step: 100 },
      { path: "values.dubins_turn_radius_m", label: "Dubins 선회반경(m)", type: "number", min: 50, step: 10 },
      { path: "values.cruise_speed_mps", label: "UAV 순항속도(m/s)", type: "number", min: 1, max: 100, step: 1 },
      { path: "values.uav_speed_weight", label: "UAV 속도 가중치", type: "number", min: 0.1, max: 5, step: 0.05 },
      { path: "values.uav_climb_rate_mps", label: "UAV 상승률(m/s)", type: "number", min: 0.1, max: 30, step: 0.1 },
      { path: "values.altitude_m", label: "AGL 오프셋(m)", type: "int", min: 10, max: 20000, step: 10 },
      { path: "values.altitude_layer_step_m", label: "AGL 레이어 간격(m)", type: "number", min: 0, max: 1000, step: 1 },
      { path: "values.sweep_line_interp_points", label: "Sweep 보간점 수", type: "int", min: 2, max: 15, step: 1 },
      { path: "values.min_sweep_len_m", label: "최소 Sweep 길이(m)", type: "number", min: 0, step: 1 },
      { path: "values.min_route_spacing_m", label: "최소 경로 간격(m)", type: "number", min: 0, step: 10 },
      { path: "values.area_dubins_entry_links_enabled", label: "Area 전환 Dubins 연결", type: "bool" },
    ],
  },
  {
    title: "Line / Area / Recon",
    fields: [
      { path: "values.global_density_scale", label: "공통 탐색 배율", type: "number", min: 0.2, max: 10, step: 0.05 },
      { path: "values.global_search_speed_weight", label: "공통 탐색 속도 가중치", type: "number", min: 0.1, max: 10, step: 0.1 },
      { path: "values.global_route_offset_scale", label: "공통 경로 오프셋 배수", type: "number", min: 0, max: 2, step: 0.05 },
      { path: "values.line_override_enabled", label: "Line 전용값 사용", type: "bool" },
      { path: "values.line_override_fov_deg", label: "Line FOV(deg)", type: "number", min: 1.2, max: 120, step: 0.1 },
      { path: "values.line_override_density_scale", label: "Line 탐색 배율", type: "number", min: 0.2, max: 10, step: 0.05 },
      { path: "values.line_override_search_speed_weight", label: "Line 속도 가중치", type: "number", min: 0.1, max: 10, step: 0.1 },
      { path: "values.line_override_route_offset_scale", label: "Line 경로 오프셋 배수", type: "number", min: 0, max: 2, step: 0.05 },
      { path: "values.area_override_enabled", label: "Area 전용값 사용", type: "bool" },
      { path: "values.area_override_fov_deg", label: "Area FOV(deg)", type: "number", min: 1.2, max: 120, step: 0.1 },
      { path: "values.area_override_density_scale", label: "Area 탐색 배율", type: "number", min: 0.2, max: 10, step: 0.05 },
      { path: "values.area_override_search_speed_weight", label: "Area 속도 가중치", type: "number", min: 0.1, max: 10, step: 0.1 },
      { path: "values.area_override_route_offset_scale", label: "Area 경로 오프셋 배수", type: "number", min: 0, max: 2, step: 0.05 },
      { path: "values.area_output_fov_scale", label: "Area 출력 FOV 배수", type: "number", min: 0.1, max: 10, step: 0.1 },
      { path: "values.area_nadir_override_enabled", label: "Area Nadir 전용값", type: "bool" },
      { path: "values.area_nadir_override_fov_deg", label: "Area Nadir FOV(deg)", type: "number", min: 0.1, max: 120, step: 0.1 },
      { path: "values.recon_override_enabled", label: "Recon 전용값", type: "bool" },
      { path: "values.recon_override_split_width_m", label: "Recon split width(m)", type: "number", min: 50, step: 10 },
      { path: "values.recon_override_fixed_fov_deg", label: "Recon FOV(deg)", type: "number", min: 0.1, max: 120, step: 0.1 },
      { path: "values.recon_override_sweep_separation_scale", label: "Recon SEP 배수", type: "number", min: 0.05, max: 1, step: 0.05 },
    ],
  },
  {
    title: "재계획 성능",
    fields: [
      { path: "values.replan_sweep_speed_scale", label: "재계획 sweep 속도 배수", type: "number", min: 0.1, max: 5, step: 0.05 },
      { path: "values.replan_variant_parallel_enabled", label: "Variant 병렬", type: "bool" },
      { path: "values.replan_current_remaining_variant_parallel_enabled", label: "Current/Remaining 병렬", type: "bool" },
      { path: "values.replan_reexecute_current_fast_path_enabled", label: "재실행 current fast path", type: "bool" },
      { path: "values.replan_reexecute_line_recon_hybrid_share_enabled", label: "Line/Recon hybrid 공유", type: "bool" },
      { path: "values.replan_variant_workers", label: "Variant workers", type: "int", min: 0, max: 16, step: 1 },
      { path: "values.replan_variant_waypoint_block_size", label: "Waypoint block size", type: "int", min: 1, step: 100 },
      { path: "values.replan_current_remaining_precompute_workers", label: "Precompute workers", type: "int", min: 0, max: 16, step: 1 },
      { path: "values.replan_store_prepare_workers", label: "Store prepare workers", type: "int", min: 0, max: 16, step: 1 },
      { path: "values.replan_store_prepare_out_of_order", label: "Store prepare out-of-order", type: "bool" },
      { path: "values.replan_store_commit_workers", label: "Store commit workers", type: "int", min: 0, max: 16, step: 1 },
      { path: "values.replan_store_json_write_workers", label: "JSON write workers", type: "int", min: 0, max: 16, step: 1 },
      { path: "values.replan_0303_aircraft_workers", label: "0303 aircraft workers", type: "int", min: 0, max: 16, step: 1 },
      { path: "values.replan_0303_dependency_parallel_enabled", label: "0303 dependency 병렬", type: "bool" },
      { path: "values.replan_0303_dependency_workers", label: "0303 dependency workers", type: "int", min: 0, max: 16, step: 1 },
      { path: "values.replan_0303_altitude_precompute_enabled", label: "0303 고도 precompute", type: "bool" },
      { path: "values.replan_dem_batch_enabled", label: "DEM batch", type: "bool" },
    ],
  },
  {
    title: "다음 협업 임무",
    fields: [
      {
        path: "values.next_collab_default_entry_strategy",
        label: "진입 전략",
        type: "select",
        options: [
          ["현재 위치", "current_position"],
          ["중간점 진입", "midpoint_to_next_start"],
          ["선회 예측", "turn_projection"],
        ],
      },
      { path: "values.next_collab_sweep_step_ratio", label: "Sweep step ratio", type: "number", min: 0.1, max: 1, step: 0.05 },
      { path: "values.next_collab_entry_tprime_target_sep_ratio", label: "T' target SEP ratio", type: "number", min: 0.1, max: 1, step: 0.05 },
      { path: "values.next_collab_entry_tprime_ratio_scale", label: "T' ratio scale", type: "number", min: 0.1, max: 5, step: 0.05 },
      { path: "values.next_collab_area_path0_trigger_sep_m", label: "Area path0 trigger SEP(m)", type: "number", min: 0, step: 100 },
      { path: "values.next_collab_area_path0_target_sep_ratio", label: "Area path0 target SEP ratio", type: "number", min: 0.05, max: 1, step: 0.05 },
      { path: "values.next_collab_turn_radius_scale", label: "선회반경 배수", type: "number", min: 0.1, max: 5, step: 0.05 },
      { path: "values.next_collab_takeover_first_step_ratio", label: "Takeover first step ratio", type: "number", min: 0.1, max: 1, step: 0.05 },
      { path: "values.next_collab_area_fov_scale", label: "Area FOV 배수", type: "number", min: 0.1, max: 5, step: 0.05 },
      { path: "values.next_collab_area_density_scale", label: "Area 탐색 배율", type: "number", min: 0.2, max: 10, step: 0.05 },
      { path: "values.next_collab_area_search_speed_scale", label: "Area sweep speed scale", type: "number", min: 0.1, max: 5, step: 0.05 },
      { path: "values.next_collab_area_gsd_margin_ratio", label: "Area GSD margin", type: "number", min: 0.1, max: 1, step: 0.05 },
      { path: "values.next_collab_auto_sweep_points", label: "Auto sweep points", type: "bool" },
      { path: "values.next_collab_sweep_points_per_leg", label: "Sweep points/leg", type: "int", min: 2, max: 9, step: 1 },
      { path: "values.next_collab_line_db_width_weight", label: "Line DB width weight", type: "number", min: 0, max: 1, step: 0.05 },
      { path: "values.next_collab_line_db_sep_weight", label: "Line DB SEP weight", type: "number", min: 0, max: 1, step: 0.05 },
      { path: "values.next_collab_line_db_fov_weight", label: "Line DB FOV weight", type: "number", min: 0, max: 1, step: 0.05 },
      { path: "values.next_collab_first_line_fov_scale", label: "첫 Line FOV 배수", type: "number", min: 0.1, max: 5, step: 0.05 },
      { path: "values.next_collab_first_line_fov_max_deg", label: "첫 Line FOV 최대(deg)", type: "number", min: 0.1, max: 120, step: 0.1 },
    ],
  },
  {
    title: "선행 임무",
    fields: [
      { path: "prior_mission.tracking_loiter_seconds", label: "추적 체공시간(s)", type: "int", min: 0, step: 5 },
      { path: "prior_mission.default_loiter_seconds", label: "기본 체공시간(s)", type: "int", min: 0, step: 5 },
      { path: "prior_mission.reinsert_loiter_seconds", label: "재삽입 체공시간(s)", type: "int", min: 0, step: 5 },
      { path: "prior_mission.approach_base_offset_m", label: "기본 접근 오프셋(m)", type: "number", min: 0, step: 10 },
      { path: "prior_mission.approach_far_offset_m", label: "원거리 접근 오프셋(m)", type: "number", min: 0, step: 10 },
      { path: "prior_mission.approach_far_trigger_distance_m", label: "원거리 전환 거리(m)", type: "number", min: 0, step: 10 },
      { path: "prior_mission.orientation_offset_m", label: "방향 지정 오프셋(m)", type: "number", min: 0, step: 10 },
      { path: "prior_mission.approach_speed_mps", label: "접근속도(m/s)", type: "number", min: 1, max: 100, step: 1 },
      { path: "prior_mission.target_speed_mps", label: "목표 WP 속도(m/s)", type: "number", min: 1, max: 100, step: 1 },
      { path: "prior_mission.resume_search_speed_scale", label: "재개 탐색속도 배수", type: "number", min: 0.1, max: 5, step: 0.05 },
    ],
  },
  {
    title: "공격 임무 / LAH",
    fields: [
      { path: "attack_mission.manned_candidate_ids", label: "공격 유인기 후보 ID", type: "list" },
      { path: "attack_mission.target_type_priority", label: "표적 타입 우선순위", type: "list" },
      {
        path: "attack_mission.weapon_type",
        label: "Fallback weaponType",
        type: "select",
        options: [
          ["1", 1],
          ["2", 2],
          ["3", 3],
        ],
      },
      { path: "attack_mission.weapon_for_target_type_1", label: "targetType 1 weapon", type: "int", min: 0, max: 9, step: 1 },
      { path: "attack_mission.weapon_for_target_type_2", label: "targetType 2 weapon", type: "int", min: 0, max: 9, step: 1 },
      { path: "attack_mission.weapon_for_target_type_3", label: "targetType 3 weapon", type: "int", min: 0, max: 9, step: 1 },
      { path: "attack_mission.weapon_for_target_type_4", label: "targetType 4 weapon", type: "int", min: 0, max: 9, step: 1 },
      { path: "attack_mission.weapon_for_target_type_5", label: "targetType 5 weapon", type: "int", min: 0, max: 9, step: 1 },
      { path: "attack_mission.weapon_for_target_type_6", label: "targetType 6 weapon", type: "int", min: 0, max: 9, step: 1 },
      { path: "attack_mission.entry_offset_m", label: "공격 진입 오프셋(m)", type: "number", min: 0, step: 10 },
      { path: "attack_mission.resume_offset_m", label: "공격 후 resume 오프셋(m)", type: "number", min: 0, step: 10 },
      { path: "attack_mission.attack_min_standoff_m", label: "최소 이격거리(m)", type: "number", min: 0, step: 10 },
      { path: "attack_mission.attack_preferred_standoff_m", label: "선호 이격거리(m)", type: "number", min: 0, step: 10 },
      { path: "attack_mission.attack_point_altitude_offset_m", label: "공격점 고도 오프셋(m)", type: "number", min: 0, step: 10 },
      { path: "attack_mission.lah_hold_seconds", label: "LAH hold(s)", type: "int", min: 0, step: 5 },
      { path: "attack_mission.lah_hold_near_resume_offset_m", label: "Hold 지점 resume 오프셋(m)", type: "number", min: 0, step: 10 },
      { path: "attack_mission.resume_search_speed_scale", label: "Resume 탐색속도 배수", type: "number", min: 0.1, max: 5, step: 0.05 },
      {
        path: "values.lah_path_mode",
        label: "LAH 경로 모드",
        type: "select",
        options: [
          ["Linear", "linear"],
          ["RL", "rl"],
        ],
      },
      { path: "values.lah_rl_hex_step", label: "LAH RL hex step", type: "int", min: 10, max: 100, step: 1 },
      { path: "values.lah_rl_area_km", label: "LAH RL area(km)", type: "number", min: 2, max: 50, step: 1 },
    ],
  },
  {
    title: "출력 / Flyover",
    fields: [
      { path: "flyover.entry_offset", label: "임무 시작점 flyover", type: "bool" },
      { path: "flyover.dubins_prefix", label: "Area Dubins prefix flyover", type: "bool" },
      { path: "flyover.last_point", label: "개별 임무 마지막점 flyover", type: "bool" },
      { path: "flyover.all_wps", label: "모든 WP flyover", type: "bool" },
    ],
  },
];

const replanToggleLabels = [
  ["input_refresh", "입력 갱신"],
  ["prior_mission", "선행 임무"],
  ["dl_risk", "DL 위험"],
  ["target_detection", "표적 탐지"],
  ["post_attack_rejoin", "공격 후 복귀"],
  ["forced_command", "강제 대기"],
  ["rtb", "RTB"],
  ["path_deviation", "경로 추종"],
  ["quality_monitor", "촬영 품질 모니터"],
  ["quality_speed", "촬영 품질 속도"],
  ["imaging_schedule", "촬영 일정"],
  ["next_collab", "다음 협업"],
  ["fuel_threshold", "연료 임계값"],
];

const replanSections = [
  {
    title: "경로 추종",
    fields: [
      { path: "path_deviation.turn_rate_threshold_dps", label: "선회율 임계값(deg/s)", type: "number", min: 0.1, step: 0.05 },
      { path: "path_deviation.turn_window_s", label: "선회 계산 창(s)", type: "number", min: 2, step: 0.5 },
      { path: "path_deviation.stale_timeout_s", label: "수신 timeout(s)", type: "number", min: 0.5, step: 0.5 },
      { path: "path_deviation.heading_move_min_m", label: "좌표 heading 최소 이동(m)", type: "number", min: 0, step: 0.5 },
      { path: "path_deviation.turn_gap_reset_s", label: "선회 추적 reset 간격(s)", type: "number", min: 0.1, step: 0.1 },
      { path: "path_deviation.spiral_window_s", label: "Orbit 감시 창(s)", type: "number", min: 5, step: 1 },
      { path: "path_deviation.spiral_min_points", label: "최소 샘플 수", type: "int", min: 3, step: 1 },
      { path: "path_deviation.center_ignore_radius_m", label: "WP 중심 무시 반경(m)", type: "number", min: 0, step: 5 },
      { path: "path_deviation.watch_angle_deg", label: "주의 각도(deg)", type: "number", min: 10, max: 360, step: 5 },
      { path: "path_deviation.warning_angle_deg", label: "경고 각도(deg)", type: "number", min: 20, max: 360, step: 5 },
      { path: "path_deviation.hold_s", label: "경고 유지시간(s)", type: "number", min: 0, step: 0.5 },
      { path: "path_deviation.release_min_distance_m", label: "해제 최소거리(m)", type: "number", min: 1, step: 10 },
      { path: "path_deviation.release_factor", label: "해제 반경 배수", type: "number", min: 1, step: 0.05 },
      { path: "path_deviation.alt_waypoint_trigger_s", label: "대체 WP trigger(s)", type: "number", min: 0, step: 0.5 },
      { path: "path_deviation.alt_waypoint_lead_time_s", label: "대체 WP lead time(s)", type: "number", min: 0, step: 0.5 },
      { path: "path_deviation.next_mission_entry_lead_time_s", label: "다음 임무 진입 lead time(s)", type: "number", min: 0, step: 0.5 },
      { path: "path_deviation.turn_radius_30_m", label: "30m/s 반경(m)", type: "number", min: 1, step: 10 },
      { path: "path_deviation.turn_radius_40_m", label: "40m/s 반경(m)", type: "number", min: 1, step: 10 },
      { path: "path_deviation.turn_radius_50_m", label: "50m/s 반경(m)", type: "number", min: 1, step: 10 },
      { path: "path_deviation.adaptive_enabled", label: "Adaptive 보정", type: "bool" },
      { path: "path_deviation.adaptive_ema_alpha", label: "Adaptive EMA alpha", type: "number", min: 0.01, max: 1, step: 0.01 },
      { path: "path_deviation.adaptive_save_interval_s", label: "Adaptive 저장 간격(s)", type: "number", min: 1, step: 1 },
    ],
  },
  {
    title: "품질 / 입력 / DL",
    fields: [
      { path: "quality_speed.lower_band_ratio", label: "품질 하한 비율", type: "number", min: 0.1, max: 1, step: 0.01 },
      { path: "quality_speed.search_speed_up_scale", label: "속도 상향 배수", type: "number", min: 0.1, max: 5, step: 0.01 },
      { path: "quality_speed.search_speed_down_scale", label: "속도 하향 배수", type: "number", min: 0.1, max: 5, step: 0.01 },
      { path: "quality_speed.min_sample_count", label: "최소 샘플 수", type: "int", min: 1, step: 1 },
      { path: "quality_speed.max_sample_count", label: "최대 샘플 수", type: "int", min: 1, step: 1 },
      { path: "quality_speed.startup_grace_sec", label: "시작 유예시간(s)", type: "number", min: 0, step: 0.5 },
      { path: "quality_speed.disabled_flight_mode", label: "비활성 flight mode", type: "int", min: 0, step: 1 },
      { path: "input_refresh.duplicate_window_ms", label: "입력 중복 방지(ms)", type: "int", min: 0, step: 50 },
      { path: "input_refresh.block_when_reexecute_active", label: "재실행 중 입력 차단", type: "bool" },
      { path: "prior_mission.dl_risk_threshold", label: "선행임무 DL 위험 임계값", type: "number", min: 0, max: 1, step: 0.01 },
      { path: "dl_risk.mean_risk_threshold", label: "DL 평균 risk 임계값", type: "number", min: 0, max: 1, step: 0.01 },
      { path: "dl_risk.cooldown_sec", label: "DL 위험 cooldown(s)", type: "number", min: 0, step: 1 },
    ],
  },
  {
    title: "명령 / RTB / 연료",
    fields: [
      { path: "forced_command.hold_delay_seconds", label: "강제대기 hold(s)", type: "number", min: 0, step: 0.5 },
      { path: "forced_command.signature_dedup_seconds", label: "명령 중복 방지(s)", type: "number", min: 0, step: 0.1 },
      { path: "rtb.unexpected_rtb_flight_mode", label: "예상 밖 RTB flight mode", type: "int", min: 0, step: 1 },
      { path: "rtb.abnormal_health_value", label: "비정상 health 값", type: "int", min: 0, step: 1 },
      { path: "rtb.fuel_warning_replan_level", label: "연료 경고 replan level", type: "int", min: 0, step: 1 },
      { path: "rtb.signal_loss_grace_ms", label: "신호 손실 유예(ms)", type: "int", min: 0, step: 100 },
      { path: "rtb.replan_hold_ms", label: "RTB hold(ms)", type: "int", min: 0, step: 100 },
      { path: "rtb.fault_unavailable_hold_ms", label: "고장/통신/장비 hold(ms)", type: "int", min: 0, step: 1000 },
      { path: "rtb.command_aircraft_id", label: "RTB 명령 항공기 ID", type: "int", min: 1, step: 1 },
      { path: "fuel_threshold.capacity_liters", label: "연료 총량(L)", type: "number", min: 0.1, step: 0.1 },
      { path: "fuel_threshold.yellow_ratio", label: "Yellow 비율", type: "number", min: 0, max: 1, step: 0.01 },
      { path: "fuel_threshold.red_ratio", label: "Red 비율", type: "number", min: 0, max: 1, step: 0.01 },
    ],
  },
  {
    title: "촬영 일정 / 다음 협업",
    fields: [
      { path: "imaging_schedule.trigger_probability", label: "촬영 트리거 확률", type: "number", min: 0, max: 1, step: 0.01 },
      { path: "imaging_schedule.imaging_operation_modes", label: "허용 operation modes", type: "list" },
      { path: "imaging_schedule.imaging_pattern_types", label: "허용 pattern types", type: "list" },
    ],
  },
  {
    title: "표적 탐지 / 공격 후 복귀",
    fields: [
      { path: "target_detection.cooldown_ms", label: "탐지 cooldown(ms)", type: "int", min: 0, step: 100 },
      { path: "target_detection.watcher_uav_ids", label: "Watcher UAV IDs", type: "list" },
      { path: "target_detection.attack_manned_ids", label: "Attack manned IDs", type: "list" },
      { path: "target_detection.target_type_priority", label: "Target type priority", type: "list" },
      { path: "target_detection.option_presets", label: "Option presets(JSON)", type: "json" },
      { path: "post_attack_rejoin.closure_cooldown_ms", label: "종료 트리거 중복 방지(ms)", type: "int", min: 0, step: 100 },
      { path: "post_attack_rejoin.min_remaining_eta_s", label: "재계획 최소 잔여 ETA(s)", type: "int", min: 0, step: 5 },
      { path: "post_attack_rejoin.rejoin_margin_s", label: "복귀 여유시간(s)", type: "int", min: 0, step: 5 },
      { path: "post_attack_rejoin.turn_radius_m", label: "복귀 선회반경(m)", type: "number", min: 1, step: 5 },
      { path: "post_attack_rejoin.default_cruise_speed_mps", label: "복귀 기본속도(m/s)", type: "number", min: 1, step: 0.5 },
      { path: "post_attack_rejoin.active_progress_skip_percent", label: "Active progress skip(%)", type: "int", min: 0, max: 100, step: 1 },
    ],
  },
  {
    title: "Replan Queue",
    fields: [
      { path: "replan_queue.active_timeout_ms", label: "Active timeout(ms)", type: "int", min: 1000, step: 500 },
      { path: "replan_queue.history_limit", label: "History limit", type: "int", min: 5, step: 1 },
      { path: "replan_queue.target_dispatch_delay_ms", label: "Target burst delay(ms)", type: "int", min: 0, step: 100 },
      { path: "replan_queue.release_on_option_info", label: "0701 수신 시 release", type: "bool" },
      { path: "replan_queue.suppress_active_target_options_on_new_detection", label: "새 탐지 시 active target option 중단", type: "bool" },
    ],
  },
];

function getByPath(root, path) {
  return path.split(".").reduce((obj, key) => (obj && Object.prototype.hasOwnProperty.call(obj, key) ? obj[key] : undefined), root);
}

function setByPath(root, path, value) {
  const keys = path.split(".");
  let obj = root;
  for (let i = 0; i < keys.length - 1; i += 1) {
    const key = keys[i];
    if (!obj[key] || typeof obj[key] !== "object" || Array.isArray(obj[key])) {
      obj[key] = {};
    }
    obj = obj[key];
  }
  obj[keys[keys.length - 1]] = value;
}

function normalizeType(value) {
  if (typeof value === "boolean") return "bool";
  if (typeof value === "number") return Number.isInteger(value) ? "int" : "number";
  if (Array.isArray(value)) return value.every((item) => Number.isInteger(Number(item))) ? "list" : "json";
  if (value && typeof value === "object") return "json";
  return "text";
}

function parseList(text) {
  return String(text || "")
    .split(/[,\s;]+/)
    .map((item) => item.trim())
    .filter(Boolean)
    .map((item) => Number(item))
    .filter((item) => Number.isFinite(item))
    .map((item) => Math.trunc(item));
}

function formatValueForInput(spec, value) {
  if (spec.type === "list") return Array.isArray(value) ? value.join(", ") : "";
  if (spec.type === "json") return JSON.stringify(value ?? null, null, 2);
  if (value === undefined || value === null) return "";
  return String(value);
}

function readFieldValue(spec, input) {
  if (spec.type === "bool") return Boolean(input.checked);
  if (spec.type === "int") return Math.trunc(Number(input.value || 0));
  if (spec.type === "number") return Number(input.value || 0);
  if (spec.type === "list") return parseList(input.value);
  if (spec.type === "json") return JSON.parse(input.value || "null");
  if (spec.type === "select") {
    return JSON.parse(input.value);
  }
  return input.value;
}

function setNumericAttrs(input, spec) {
  if (spec.min !== undefined) input.min = spec.min;
  if (spec.max !== undefined) input.max = spec.max;
  if (spec.step !== undefined) input.step = spec.step;
}

function createSwitch(checked, onChange) {
  const wrapper = document.createElement("label");
  wrapper.className = "switch";
  const input = document.createElement("input");
  input.type = "checkbox";
  input.checked = Boolean(checked);
  input.addEventListener("change", () => onChange(input.checked));
  const slider = document.createElement("span");
  slider.className = "slider";
  wrapper.append(input, slider);
  return wrapper;
}

function createField(root, spec) {
  const row = document.createElement("div");
  row.className = `field ${spec.type === "bool" ? "switch-field" : ""}`;
  const label = document.createElement("label");
  label.textContent = spec.label;
  row.appendChild(label);

  const value = getByPath(root, spec.path);
  let input;

  if (spec.type === "bool") {
    input = createSwitch(value, (checked) => {
      setByPath(root, spec.path, checked);
    });
    row.appendChild(input);
    return row;
  }

  if (spec.type === "select") {
    input = document.createElement("select");
    for (const [text, data] of spec.options || []) {
      const option = document.createElement("option");
      option.textContent = text;
      option.value = JSON.stringify(data);
      if (JSON.stringify(data) === JSON.stringify(value)) option.selected = true;
      input.appendChild(option);
    }
  } else if (spec.type === "json") {
    input = document.createElement("textarea");
    input.spellcheck = false;
  } else {
    input = document.createElement("input");
    input.type = spec.type === "number" || spec.type === "int" ? "number" : "text";
    if (input.type === "number") setNumericAttrs(input, spec);
  }

  input.value = formatValueForInput(spec, value);
  const eventName = spec.type === "text" || spec.type === "list" || spec.type === "json" ? "change" : "input";
  input.addEventListener(eventName, () => {
    try {
      setByPath(root, spec.path, readFieldValue(spec, input));
      setStatus("수정됨", "");
    } catch (error) {
      setStatus(`${spec.label}: ${error.message}`, "error");
    }
  });
  row.appendChild(input);
  return row;
}

function createSection(root, section) {
  const card = document.createElement("section");
  card.className = "section-card";
  const head = document.createElement("div");
  head.className = "section-head";
  const title = document.createElement("h2");
  title.textContent = section.title;
  head.appendChild(title);
  card.appendChild(head);
  const grid = document.createElement("div");
  grid.className = "field-grid";
  for (const spec of section.fields) {
    grid.appendChild(createField(root, spec));
  }
  card.appendChild(grid);
  return card;
}

function listedPaths(sections) {
  return new Set(sections.flatMap((section) => section.fields.map((field) => field.path)));
}

function extraFields(root, prefix, listed) {
  const group = getByPath(root, prefix);
  if (!group || typeof group !== "object" || Array.isArray(group)) return [];
  return Object.keys(group)
    .filter((key) => !key.startsWith("_"))
    .map((key) => `${prefix}.${key}`)
    .filter((path) => !listed.has(path))
    .sort()
    .map((path) => ({
      path,
      label: path,
      type: normalizeType(getByPath(root, path)),
    }));
}

function renderMission() {
  const panel = $("#missionPanel");
  panel.innerHTML = "";
  for (const section of missionSections) {
    panel.appendChild(createSection(state.mission, section));
  }
  const listed = listedPaths(missionSections);
  const extras = [
    ...extraFields(state.mission, "values", listed),
    ...extraFields(state.mission, "prior_mission", listed),
    ...extraFields(state.mission, "attack_mission", listed),
    ...extraFields(state.mission, "flyover", listed),
  ];
  if (extras.length) {
    panel.appendChild(createSection(state.mission, { title: "기타 임무계획 값", fields: extras }));
  }
}

function renderReplanToggles(panel) {
  const card = document.createElement("section");
  card.className = "section-card";
  const head = document.createElement("div");
  head.className = "section-head";
  const title = document.createElement("h2");
  title.textContent = "기능 ON / OFF";
  const tools = document.createElement("div");
  tools.className = "actions";
  const restoreBtn = document.createElement("button");
  restoreBtn.type = "button";
  restoreBtn.textContent = "권장값 복원";
  restoreBtn.addEventListener("click", restoreRecommendedReplan);
  const saveRecommendedBtn = document.createElement("button");
  saveRecommendedBtn.type = "button";
  saveRecommendedBtn.textContent = "현재값을 권장값으로 저장";
  saveRecommendedBtn.addEventListener("click", saveRecommendedReplan);
  tools.append(restoreBtn, saveRecommendedBtn);
  head.append(title, tools);
  card.appendChild(head);

  const grid = document.createElement("div");
  grid.className = "toggle-grid";
  for (const [key, label] of replanToggleLabels) {
    const item = document.createElement("div");
    item.className = "toggle-card";
    const text = document.createElement("span");
    text.textContent = label;
    const control = createSwitch(getByPath(state.replan, `toggles.${key}`), (checked) => {
      setByPath(state.replan, `toggles.${key}`, checked);
      setStatus("수정됨", "");
    });
    item.append(text, control);
    grid.appendChild(item);
  }
  card.appendChild(grid);
  panel.appendChild(card);
}

function renderReplan() {
  const panel = $("#replanPanel");
  panel.innerHTML = "";
  renderReplanToggles(panel);
  for (const section of replanSections) {
    panel.appendChild(createSection(state.replan, section));
  }
  const listed = listedPaths(replanSections);
  for (const group of Object.keys(state.replan || {}).sort()) {
    if (group === "version" || group === "toggles") continue;
    const fields = extraFields(state.replan, group, listed);
    if (fields.length) panel.appendChild(createSection(state.replan, { title: `기타 ${group}`, fields }));
  }
}

function renderAll() {
  renderMission();
  renderReplan();
}

function syncRaw() {
  return;
}

function applyRawLegacy(kind) {
  try {
    renderAll();
    setStatus("JSON을 화면에 반영했습니다.", "ok");
  } catch (error) {
    setStatus(`JSON 오류: ${error.message}`, "error");
  }
}

function setStatus(text, type) {
  const bar = $(".statusbar");
  $("#statusText").textContent = text;
  bar.classList.remove("ok", "error");
  if (type) bar.classList.add(type);
}

async function loadSettings() {
  setStatus("불러오는 중", "");
  const res = await fetch("/api/settings", { cache: "no-store" });
  if (!res.ok) throw new Error(await res.text());
  const payload = await res.json();
  state.mission = payload.mission;
  state.replan = payload.replan;
  state.recommendedReplan = payload.recommendedReplan;
  state.paths = payload.paths || {};
  $("#pathText").textContent = `${state.paths.mission || ""} / ${state.paths.replan || ""}`;
  renderAll();
  setStatus("준비됨", "ok");
}

async function saveSettingsLegacy() {
  try {
  } catch (error) {
    setStatus(`저장 전 JSON 오류: ${error.message}`, "error");
    return;
  }
  setStatus("저장 중", "");
  const res = await fetch("/api/settings", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mission: state.mission, replan: state.replan }),
  });
  const payload = await res.json();
  if (!res.ok) {
    setStatus(payload.error || "저장 실패", "error");
    return;
  }
  state.mission = payload.mission || state.mission;
  state.replan = payload.replan || state.replan;
  state.paths = payload.paths || state.paths;
  renderAll();
  setStatus("저장 완료", "ok");
}

function restoreRecommendedReplan() {
  if (!state.recommendedReplan) {
    setStatus("권장값을 찾을 수 없습니다.", "error");
    return;
  }
  state.replan = JSON.parse(JSON.stringify(state.recommendedReplan));
  renderAll();
  setStatus("권장값을 화면에 반영했습니다.", "ok");
}

async function saveRecommendedReplanLegacy() {
  try {
  } catch (error) {
    setStatus(`권장값 저장 전 JSON 오류: ${error.message}`, "error");
    return;
  }
  const res = await fetch("/api/replan/recommended", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ replan: state.replan }),
  });
  const payload = await res.json();
  if (!res.ok) {
    setStatus(payload.error || "권장값 저장 실패", "error");
    return;
  }
  state.recommendedReplan = payload.recommendedReplan;
  setStatus("권장값 저장 완료", "ok");
}

function bindTabsLegacy() {
  for (const tab of document.querySelectorAll(".tab")) {
    tab.addEventListener("click", () => {
      for (const node of document.querySelectorAll(".tab")) node.classList.remove("active");
      for (const panel of document.querySelectorAll(".panel")) panel.classList.remove("active");
      tab.classList.add("active");
      $(`#${tab.dataset.tab}Panel`).classList.add("active");
    });
  }
}

function bindActions() {
  $("#reloadBtn").addEventListener("click", () => loadSettings().catch((error) => setStatus(error.message, "error")));
  $("#saveBtn").addEventListener("click", saveSettings);
}

async function saveSettings() {
  setStatus("저장 중", "");
  const res = await fetch("/api/settings", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mission: state.mission, replan: state.replan }),
  });
  const payload = await res.json();
  if (!res.ok) {
    setStatus(payload.error || "저장 실패", "error");
    return;
  }
  state.mission = payload.mission || state.mission;
  state.replan = payload.replan || state.replan;
  state.paths = payload.paths || state.paths;
  renderAll();
  setStatus("저장 완료", "ok");
}

async function saveRecommendedReplan() {
  const res = await fetch("/api/replan/recommended", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ replan: state.replan }),
  });
  const payload = await res.json();
  if (!res.ok) {
    setStatus(payload.error || "권장값 저장 실패", "error");
    return;
  }
  state.recommendedReplan = payload.recommendedReplan;
  setStatus("권장값 저장 완료", "ok");
}

bindActions();
loadSettings().catch((error) => setStatus(error.message, "error"));
