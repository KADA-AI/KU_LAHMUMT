from __future__ import annotations

import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
NODE = shutil.which("node")


def _node_command(script: str) -> list[str]:
    assert NODE is not None
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
    return command


def test_distance_measurement_is_wired_into_the_sim_map() -> None:
    index = (ROOT / "modules/sim/web/index.html").read_text(encoding="utf-8")
    app = (ROOT / "modules/sim/web/app.js").read_text(encoding="utf-8")
    style = (ROOT / "modules/sim/web/style.css").read_text(encoding="utf-8")
    tool = (
        ROOT / "modules/sim/web/js/distance_measurement.js"
    ).read_text(encoding="utf-8")

    assert 'id="toggle-distance-measurement"' in index
    assert "거리 측정" in index
    assert 'from "./js/distance_measurement.js' in app
    assert "initDistanceMeasurement(map" in app
    assert "sim-map-distance-measure" in style
    assert 'container?.addEventListener("click", onMapClickCapture, true)' in tool
    assert (
        'container?.addEventListener("contextmenu", onMapContextMenuCapture, true)'
        in tool
    )
    assert "stopImmediatePropagation" in tool
    assert "points.length < 2" in tool
    assert "finalized && !active" in tool


@pytest.mark.skipif(
    NODE is None,
    reason="Node.js is required for the browser-module contract test",
)
def test_distance_and_polygon_feature_contract() -> None:
    script = textwrap.dedent(
        """
        import assert from "node:assert/strict";
        import {
          buildMeasurementFeatureCollection,
          distanceMeters,
          formatDistance,
        } from "./modules/sim/web/js/distance_measurement.js";

        const oneDegree = distanceMeters([127, 37], [127, 38]);
        assert.ok(oneDegree > 111000 && oneDegree < 111300);
        assert.equal(formatDistance(999.4), "999 m");
        assert.equal(formatDistance(1500), "1.50 km");

        const line = buildMeasurementFeatureCollection([
          [127.0, 37.0],
          [127.01, 37.0],
        ]);
        assert.equal(
          line.features.filter((feature) => feature.properties.kind === "measurement-area").length,
          0,
        );
        assert.equal(
          line.features.filter((feature) => feature.properties.kind === "measurement-line").length,
          1,
        );
        assert.equal(
          line.features.filter((feature) => feature.properties.kind === "measurement-label").length,
          1,
        );

        const triangle = buildMeasurementFeatureCollection([
          [127.0, 37.0],
          [127.01, 37.0],
          [127.01, 37.01],
        ]);
        const polygon = triangle.features.find(
          (feature) => feature.properties.kind === "measurement-area",
        );
        const boundary = triangle.features.find(
          (feature) => feature.properties.kind === "measurement-line",
        );
        const labels = triangle.features.filter(
          (feature) => feature.properties.kind === "measurement-label",
        );
        assert.ok(polygon);
        assert.deepEqual(polygon.geometry.coordinates[0][0], polygon.geometry.coordinates[0][3]);
        assert.equal(boundary.geometry.coordinates.length, 4);
        assert.equal(labels.length, 3);
        assert.ok(labels[2].properties.distanceM > 0);
        """
    )
    subprocess.run(
        _node_command(script),
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )


@pytest.mark.skipif(
    NODE is None,
    reason="Node.js is required for the browser-module interaction test",
)
def test_click_click_right_click_finalizes_and_next_click_clears() -> None:
    script = textwrap.dedent(
        """
        import assert from "node:assert/strict";
        import { initDistanceMeasurement } from "./modules/sim/web/js/distance_measurement.js";

        class FakeClassList {
          constructor() { this.values = new Set(); }
          toggle(name, enabled) {
            if (enabled) this.values.add(name);
            else this.values.delete(name);
          }
          remove(name) { this.values.delete(name); }
          contains(name) { return this.values.has(name); }
        }

        class FakeElement {
          constructor() {
            this.listeners = new Map();
            this.classList = new FakeClassList();
            this.attributes = new Map();
          }
          addEventListener(type, handler) {
            const handlers = this.listeners.get(type) || [];
            handlers.push(handler);
            this.listeners.set(type, handlers);
          }
          removeEventListener(type, handler) {
            const handlers = this.listeners.get(type) || [];
            this.listeners.set(type, handlers.filter((entry) => entry !== handler));
          }
          dispatch(type, event) {
            for (const handler of this.listeners.get(type) || []) handler(event);
          }
          setAttribute(name, value) { this.attributes.set(name, value); }
          contains(target) { return target === this; }
          getBoundingClientRect() { return { left: 10, top: 20 }; }
        }

        const container = new FakeElement();
        const canvas = new FakeElement();
        const button = new FakeElement();
        const documentRef = new FakeElement();
        const sources = new Map();
        const layers = new Map();
        const mapListeners = new Map();
        const map = {
          getContainer: () => container,
          getCanvas: () => canvas,
          isStyleLoaded: () => true,
          getSource: (id) => sources.get(id),
          addSource: (id, spec) => {
            sources.set(id, {
              data: spec.data,
              setData(data) { this.data = data; },
            });
          },
          getLayer: (id) => layers.get(id),
          addLayer: (layer) => layers.set(layer.id, layer),
          unproject: ([x, y]) => ({ lng: 127 + x / 10000, lat: 37 + y / 10000 }),
          on: (type, handler) => mapListeners.set(type, handler),
          off: (type) => mapListeners.delete(type),
        };
        const eventAt = (x, y) => ({
          button: 0,
          clientX: x,
          clientY: y,
          target: { closest: () => null },
          preventDefault() {},
          stopPropagation() {},
          stopImmediatePropagation() {},
        });

        const controller = initDistanceMeasurement(map, { button, documentRef });
        controller.start();
        container.dispatch("click", eventAt(110, 120));
        container.dispatch("click", eventAt(210, 220));
        assert.equal(controller.getPoints().length, 2);
        assert.equal(controller.isActive(), true);

        container.dispatch("contextmenu", eventAt(210, 220));
        assert.equal(controller.isActive(), false);
        assert.equal(controller.isFinalized(), true);
        assert.equal(
          controller.getFeatureCollection().features.filter(
            (feature) => feature.properties.kind === "measurement-label",
          ).length,
          1,
        );

        container.dispatch("click", eventAt(310, 320));
        assert.equal(controller.isFinalized(), false);
        assert.equal(controller.getPoints().length, 0);
        assert.equal(controller.getFeatureCollection().features.length, 0);
        controller.destroy();
        """
    )
    subprocess.run(
        _node_command(script),
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
