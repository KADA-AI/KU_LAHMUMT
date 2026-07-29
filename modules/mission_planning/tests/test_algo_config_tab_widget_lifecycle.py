from __future__ import annotations

from modules.mission_planning.ui.algo_config_tab import (
    ALL_FIELD_SPECS,
    ATTACK_GROUPS,
    GENERAL_GROUPS,
    LAH_GROUPS,
    NEXT_COLLAB_FIELD_IDS,
    NEXT_COLLAB_GROUPS,
    NEXT_COLLAB_LAYOUT_COLUMNS,
    OPERATION_FIELD_IDS,
    OPERATION_GROUPS,
    PRIOR_GROUPS,
    _validate_group_layout_columns,
)


def _field_ids(groups) -> list[str]:
    return [
        f"{spec.section}.{spec.key}"
        for _title, _note, specs in groups
        for spec in specs
    ]


def test_next_collab_layout_places_every_group_exactly_once() -> None:
    group_titles = tuple(title for title, _note, _specs in NEXT_COLLAB_GROUPS)

    _validate_group_layout_columns(
        group_titles,
        NEXT_COLLAB_LAYOUT_COLUMNS,
        layout_name="test-next-collaboration",
    )

    placed_titles = [
        title
        for column in NEXT_COLLAB_LAYOUT_COLUMNS
        for title in column
    ]
    assert len(placed_titles) == len(set(placed_titles))
    assert set(placed_titles) == set(group_titles)


def test_visible_setting_fields_have_no_duplicate_widget_ids() -> None:
    excluded_common_ids = OPERATION_FIELD_IDS | NEXT_COLLAB_FIELD_IDS
    common_detail_ids = [
        field_id
        for field_id in _field_ids(GENERAL_GROUPS)
        if field_id not in excluded_common_ids
    ]
    visible_field_ids = [
        *_field_ids(OPERATION_GROUPS),
        *common_detail_ids,
        *_field_ids(NEXT_COLLAB_GROUPS),
        *_field_ids(PRIOR_GROUPS),
        *_field_ids(ATTACK_GROUPS),
        *_field_ids(LAH_GROUPS),
    ]
    all_field_ids = {
        f"{spec.section}.{spec.key}"
        for spec in ALL_FIELD_SPECS
    }

    assert len(visible_field_ids) == len(set(visible_field_ids))
    assert set(visible_field_ids) == all_field_ids
