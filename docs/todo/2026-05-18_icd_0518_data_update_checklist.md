# 2026-05-18 ICD 0518 Data Update Checklist

Scope: reflect `resource/nFusion 파일_0518` data field changes in `modules/common/generator`, `modules/common/push`, and `modules/common/receive`.

## Source ICD Diff

- [x] Compare `resource/nFusion 파일_0316` and `resource/nFusion 파일_0518`.
- [x] Confirm `0201 regionType` is already present in Generator/Push/Receive.
- [x] Confirm `0001.contents` byte length changed from 60 to 128.
- [x] Confirm `0305.replanReason` byte length changed from variable/legacy 60 handling to 128.
- [x] Confirm `0401 UnmannedInfo.onMission` changed to `flying`.
- [x] Confirm `0401 SensorInfo.filming` was added.
- [x] Confirm `0805.eventType` keeps the same field shape and only expands enum values.

## Implementation

- [x] Update shared string limit handling for 128-byte human text fields.
- [x] Update `message0001_generator.py` and `message0001_push.py` to emit/truncate `contents` at 128 bytes.
- [x] Update `message0305_generator.py` and `message0305_push.py` to emit/truncate `replanReason` at 128 bytes.
- [x] Update `message0401_generator.py` to generate `unmannedInfo.flying` and `sensorInfo.filming`.
- [x] Update `message0401_push.py` to accept new `flying`/`filming` fields and map legacy `onMission` to `flying` for compatibility.
- [x] Update `message0401_receiver.py` to deserialize `flying`/`filming` and retain `onMission` compatibility for existing internal consumers.
- [x] Update `message0805_generator.py` to generate valid 0518 `eventType` values.
- [x] Verify `message0201_*` still contain `regionType` with no further change needed.

## Verification

- [x] Run text search for old `0401 onMission` mapper locations and verify compatibility handling.
- [x] Run Python compile check for modified Python files.
- [x] Summarize remaining non-data items, if any.
