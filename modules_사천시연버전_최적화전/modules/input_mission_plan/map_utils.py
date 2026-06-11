# map_utils.py
# Folium 지도를 파일로 저장하고, QWebChannel 스크립트를 주입

import os
import folium

def write_map_html(html_path: str = "map.html",
                   center=(37.5, 127.0),
                   zoom=12) -> str:
    """
    함수: write_map_html
    역할: Folium 지도를 html_path에 저장하고, QWebChannel + Leaflet 가시화 유틸(JS) 주입
    반환: 저장된 HTML의 절대경로
    """
    import os
    import folium

    fmap = folium.Map(location=center, zoom_start=zoom, control_scale=True)
    folium.LatLngPopup().add_to(fmap)  # 디버그용(기본 팝업)

    # HTML 저장
    fmap.save(html_path)
    abs_path = os.path.abspath(html_path)

    # QWebChannel + 가시화 유틸(JS) 주입
    inject = r"""
<script src="qrc:///qtwebchannel/qwebchannel.js"></script>
<script>
(function(){
  new QWebChannel(qt.webChannelTransport, function(ch) {
    const bridge = ch.objects.bridge;

    // Folium이 만든 Leaflet Map 객체 찾기
    let mp = null;
    for (let k in window) {
      if (window[k] && window[k] instanceof L.Map) { mp = window[k]; break; }
    }
    if (!mp) return;

    // 지도 클릭 → PyQt 슬롯 호출 (기존 기능 유지)
    mp.on('click', function(e) {
      try { bridge.sendPoint(e.latlng.lat, e.latlng.lng); }
      catch(err) { console.error('bridge.sendPoint failed:', err); }
    });

    // ====== 가시화 레이어 & 드로잉 유틸 ======
    const gPreview   = L.layerGroup().addTo(mp);
    const gCommitted = L.layerGroup().addTo(mp);

    let curLine = null;     // 진행 중인 선
    let curPoly = null;     // 진행 중인 폴리곤

    // preview -> committed로 복제 후 preview 비우기
    function commitPreview() {
      const layers = gPreview.getLayers().slice();
      gPreview.clearLayers();
      for (const l of layers) {
        let nl = null;
        if (l instanceof L.Marker || l instanceof L.CircleMarker) {
          nl = L.circleMarker(l.getLatLng(), l.options);
        } else if (l instanceof L.Polygon) {
          nl = L.polygon(l.getLatLngs(), l.options);
        } else if (l instanceof L.Polyline) {
          nl = L.polyline(l.getLatLngs(), l.options);
        }
        if (nl) { nl.addTo(gCommitted); }
      }
    }

    // 전역 네임스페이스로 노출
    window.__cmpk = window.__cmpk || {};
    window.__cmpk.preview = {
      clear: () => gPreview.clearLayers(),
      commit: () => commitPreview(),
    };

    window.__cmpk.draw = {
      addPoint: function(lat, lon, alt) {
        const m = L.circleMarker([lat, lon], {radius: 4, weight: 2});
        m.bindTooltip(`(${lat.toFixed(6)}, ${lon.toFixed(6)}) alt=${alt}`, {sticky:true});
        m.addTo(gPreview);
      },
      lineStart: function(width) {
        if (curLine) { gPreview.removeLayer(curLine); }
        curLine = L.polyline([], {weight: 3}).addTo(gPreview);
        curLine._cmpkWidth = width;
      },
      lineAdd: function(lat, lon, alt) {
        if (!curLine) return;
        curLine.addLatLng([lat, lon]);
        L.circleMarker([lat, lon], {radius:3}).addTo(gPreview);
      },
      lineEnd: function() {
        curLine = null;
      },
      areaStart: function(isHole) {
        if (curPoly) { gPreview.removeLayer(curPoly); }
        curPoly = L.polygon([], {
          weight: 2,
          fillOpacity: isHole ? 0.0 : 0.2,
          dashArray: isHole ? '4 4' : null
        }).addTo(gPreview);
        curPoly._cmpkIsHole = !!isHole;
      },
      areaAdd: function(lat, lon, alt) {
        if (!curPoly) return;
        curPoly.addLatLng([lat, lon]);
        L.circleMarker([lat, lon], {radius:3}).addTo(gPreview);
      },
      areaEnd: function() {
        curPoly = null;
      }
    };
  });
})();
</script>
"""
    with open(abs_path, "r+", encoding="utf-8") as f:
        html = f.read()
        if "qwebchannel.js" not in html or "window.__cmpk" not in html:
            f.seek(0)
            f.write(html.replace("</body>", inject + "</body>"))
            f.truncate()
    return abs_path