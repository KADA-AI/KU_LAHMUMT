from __future__ import annotations

import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
NODE = shutil.which("node")


@pytest.mark.skipif(NODE is None, reason="Node.js is required for this frontend contract")
def test_completed_area_children_are_removed_from_owner_feature_immediately() -> None:
    script = textwrap.dedent(
        """
        import assert from "node:assert/strict";
        import { buildAreaFeatures } from "./modules/sim/web/js/mission_paths.js";

        const area = (x0, x1) => ({
          isHole: false,
          coordinateList: [
            { longitude: x0, latitude: 0 },
            { longitude: x1, latitude: 0 },
            { longitude: x1, latitude: 1 },
            { longitude: x0, latitude: 1 },
          ],
        });
        const mission = (missionId, pathId, x0, x1, isDone) => ({
          individualMissionID: missionId,
          pathID: pathId,
          isDone,
          relatedMission: { inputMissionID: 5 },
          individualMissionInfo: { areaList: [area(x0, x1)] },
        });
        const payload = (missions) => ({
          individualMissionPlans: [
            { aircraftID: 6, individualMissionList: missions },
          ],
          flightPaths: [],
        });

        const partlyDone = buildAreaFeatures(
          payload([
            mission(85, 18, 0, 1, true),
            mission(86, 19, 1, 2, true),
            mission(87, 20, 2, 3, false),
          ]),
          { UAV3: "#00ffff" },
          null,
        );
        assert.equal(partlyDone.length, 1);
        assert.equal(partlyDone[0].properties.ownerKey, "6:5");
        assert.equal(partlyDone[0].properties.missionIds, "87");
        assert.equal(partlyDone[0].properties.pathIds, "20");
        assert.equal(partlyDone[0].properties.areaPartCount, 1);

        const allDone = buildAreaFeatures(
          payload([
            mission(85, 18, 0, 1, true),
            mission(86, 19, 1, 2, true),
          ]),
          { UAV3: "#00ffff" },
          null,
        );
        assert.deepEqual(allDone, []);
        """
    )

    version = subprocess.run(
        [NODE, "-p", "process.versions.node"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    command = [NODE]
    if int(version.split(".", 1)[0]) < 22:
        command.append("--experimental-default-type=module")
    command.extend(["--input-type=module", "-e", script])
    subprocess.run(
        command,
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )


@pytest.mark.skipif(NODE is None, reason="Node.js is required for this frontend contract")
def test_boundary_guard_wrap_revives_static_done_areas_and_sweeps() -> None:
    script = textwrap.dedent(
        """
        import assert from "node:assert/strict";
        import {
          buildAreaFeatures,
          buildBoundaryGuardLiveStateIndex,
          buildSweepFeatures,
        } from "./modules/sim/web/js/mission_paths.js";

        const area = (x0, x1) => ({
          isHole: false,
          coordinateList: [
            { longitude: x0, latitude: 0 },
            { longitude: x1, latitude: 0 },
            { longitude: x1, latitude: 1 },
            { longitude: x0, latitude: 1 },
          ],
        });
        const waypoint = (waypointID, longitude) => ({
          waypointID,
          isDone: true,
          filmingProperty: {
            lineSearch: {
              coordinateList: [
                { longitude, latitude: 0, altitude: 100 },
                { longitude, latitude: 1, altitude: 100 },
              ],
            },
          },
        });
        const path = (pathID, sequence, waypointID, longitude) => ({
          pathID,
          aircraftID: 4,
          boundaryGuardLoop: true,
          boundaryGuardSetID: "guard:4",
          boundaryGuardSequence: sequence,
          boundaryGuardSequenceCount: 2,
          waypointList: [waypoint(waypointID, longitude)],
        });
        const mission = (individualMissionID, pathID, x0, x1) => ({
          individualMissionID,
          pathID,
          isDone: true,
          relatedMission: { inputMissionID: 5 },
          individualMissionInfo: { areaList: [area(x0, x1)] },
        });
        const payload = {
          individualMissionPlans: [{
            aircraftID: 4,
            individualMissionList: [
              mission(1, 41, 0, 1),
              mission(2, 42, 1, 2),
            ],
          }],
          flightPaths: [
            path(41, 1, 101, 0.5),
            path(42, 2, 201, 1.5),
          ],
        };
        const liveState = (sequence, currentWaypointID) => ({
          vehicles: {
            UAV1: {
              currentWaypointID,
              boundaryGuardLoopActive: true,
              boundaryGuardSetID: "guard:4",
              boundaryGuardCycleCount: 1,
              boundaryGuardSequence: sequence,
              boundaryGuardSequenceCount: 2,
            },
          },
        });
        const colors = { UAV1: "#00ffff" };

        let live = buildBoundaryGuardLiveStateIndex(liveState(1, 101));
        let current = new Map([["UAV1", 101]]);
        let areas = buildAreaFeatures(payload, colors, null, current, live);
        assert.equal(areas.length, 1);
        assert.equal(areas[0].properties.areaPartCount, 2);
        assert.equal(areas[0].properties.isDone, 0);
        assert.equal(areas[0].properties.boundaryGuardCycleCount, 1);
        let sweepLines = buildSweepFeatures(
          payload, colors, null, null, current, live,
        ).filter((feature) => feature.properties.featureKind === "line");
        assert.deepEqual(
          sweepLines.map((feature) => feature.properties.boundaryGuardStatus),
          ["active", "planned"],
        );
        assert.deepEqual(
          sweepLines.map((feature) => feature.properties.isDone),
          [0, 0],
        );

        live = buildBoundaryGuardLiveStateIndex(liveState(2, 201));
        current = new Map([["UAV1", 201]]);
        areas = buildAreaFeatures(payload, colors, null, current, live);
        assert.equal(areas.length, 1);
        assert.equal(areas[0].properties.areaPartCount, 1);
        assert.equal(areas[0].properties.pathIds, "42");
        sweepLines = buildSweepFeatures(
          payload, colors, null, null, current, live,
        ).filter((feature) => feature.properties.featureKind === "line");
        assert.deepEqual(
          sweepLines.map((feature) => feature.properties.boundaryGuardStatus),
          ["completed", "active"],
        );
        assert.deepEqual(
          sweepLines.map((feature) => feature.properties.isDone),
          [1, 0],
        );
        """
    )

    version = subprocess.run(
        [NODE, "-p", "process.versions.node"],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    command = [NODE]
    if int(version.split(".", 1)[0]) < 22:
        command.append("--experimental-default-type=module")
    command.extend(["--input-type=module", "-e", script])
    subprocess.run(
        command,
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
