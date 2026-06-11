export class ZoomResetControl {
  constructor(onReset) {
    this._onReset = onReset;
    this._container = null;
    this._handleReset = () => {
      if (this._onReset) {
        this._onReset();
      }
    };
  }

  onAdd(mapRef) {
    this._map = mapRef;
    const container = document.createElement("div");
    container.className = "maplibregl-ctrl maplibregl-ctrl-group zoom-reset-control";

    const zoomIn = document.createElement("button");
    zoomIn.type = "button";
    zoomIn.className = "zoom-reset-btn";
    zoomIn.textContent = "+";
    zoomIn.title = "Zoom in";
    zoomIn.addEventListener("click", () => this._map.zoomIn());

    const zoomOut = document.createElement("button");
    zoomOut.type = "button";
    zoomOut.className = "zoom-reset-btn";
    zoomOut.textContent = "-";
    zoomOut.title = "Zoom out";
    zoomOut.addEventListener("click", () => this._map.zoomOut());

    const reset = document.createElement("button");
    reset.type = "button";
    reset.className = "zoom-reset-btn zoom-reset-btn-reset";
    reset.textContent = "O";
    reset.title = "Reset view";
    reset.addEventListener("click", this._handleReset);

    container.appendChild(zoomIn);
    container.appendChild(zoomOut);
    container.appendChild(reset);

    this._container = container;
    return container;
  }

  onRemove() {
    if (this._container && this._container.parentNode) {
      this._container.parentNode.removeChild(this._container);
    }
    this._map = undefined;
  }
}
