# gui_full.py
import sys, os, random, time, json, folium
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QFormLayout, QLineEdit, QPushButton, QTextEdit, QTabWidget, QFrame,
    QLabel, QComboBox, QDialog, QDialogButtonBox, QGridLayout, QDoubleSpinBox)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWebChannel import QWebChannel
from branca.colormap import linear

# ───────────────────────────── 공통 헬퍼 ─────────────────────────────
def rand_coord():
    return {"latitude":round(random.uniform(-90,90),6),
            "longitude":round(random.uniform(-180,180),6),
            "altitude":round(random.uniform(50,500),1)}

def make_individual_mission(mid:int)->dict:
    info={"individualMissionType":0,"patternType":0,"autoZoomIn":True,
          "coordinateList":[],"lineList":[],"areaList":[],
          "targetID":0,"relatedIndividualMissionIDList":[]}
    return {"individualMissionID":mid,"isDone":0,
            "relatedMission":{"relatedMissionType":1,
                              "inputMissionID":0,"priorMissionID":None},
            "individualMissionInfo":info,"pathID":f"PATH-{mid:04d}"}

def add_mission_shapes(fmap, missions):
    # 항공기 별로 고유 색상
    color_scale = linear.Set1_09.scale(0, 8)   # 최대 9종 색
    ac_colors = {}
    for m in missions:
        aid = m.get("aircraftID", 0)
        if aid not in ac_colors:
            ac_colors[aid] = color_scale(len(ac_colors))
        color = ac_colors[aid]

        info = m["individualMissionInfo"]
        mtype = info["individualMissionType"]

        # 1) 영역 수색 -> Polygon
        if mtype == 1 and info["areaList"]:
            for area in info["areaList"]:
                coords = [(c["latitude"], c["longitude"])
                          for c in area["coordinateList"]]
                folium.Polygon(locations=coords,
                               color=color, weight=2,
                               fill=True, fill_opacity=0.2,
                               tooltip=f"A/C {aid} : Area").add_to(fmap)

        # 2) 통로 정찰 -> Line + 폭 표시(간단하게 PolyLine 두께로 표현)
        elif mtype == 2 and info["lineList"]:
            for line in info["lineList"]:
                coords = [(c["latitude"], c["longitude"])
                          for c in line["coordinateList"]]
                folium.PolyLine(locations=coords,
                                color=color, weight=line["width"],
                                opacity=0.6,
                                tooltip=f"A/C {aid} : Corridor").add_to(fmap)

        # 3) 이동 -> 궤적 연결선
        elif mtype == 3 and info["coordinateList"]:
            coords = [(c["latitude"], c["longitude"])
                      for c in info["coordinateList"]]
            folium.PolyLine(locations=coords,
                            color=color, weight=3,
                            dash_array="5,10",
                            tooltip=f"A/C {aid} : Route").add_to(fmap)
            
# ─── 지도-클릭 브릿지 ───
class MapBridge(QObject):
    pointClicked = pyqtSignal(float, float)   # Python 내부용 시그널

    # JS가 호출할 슬롯
    @pyqtSlot(float, float)
    def sendPoint(self, lat, lon):
        self.pointClicked.emit(lat, lon)      # Wizard 쪽 collect()로 전달

bridge = MapBridge()

# ─────────────────── 위저드 다이얼로그(0302) ───────────────────
class MissionWizard(QDialog):
    missionSaved = pyqtSignal(dict)
    def __init__(self, next_id:int, mission_plan_id:str, parent=None):
        super().__init__(parent); self.setWindowTitle("New Individual Mission")
        self.resize(450,260)
        self.mid=next_id; self.mpid=mission_plan_id
        self.coords=[]

        grid=QGridLayout(self)
        grid.addWidget(QLabel("IndividualMissionID"),0,0); grid.addWidget(QLabel(str(next_id)),0,1)
        grid.addWidget(QLabel("IsDone"),1,0);              grid.addWidget(QLabel("0"),1,1)
        grid.addWidget(QLabel("RelatedMissionType"),2,0);  grid.addWidget(QLabel("1"),2,1)
        grid.addWidget(QLabel("InputMissionID"),3,0);      grid.addWidget(QLabel(mission_plan_id),3,1)

        grid.addWidget(QLabel("IndividualMissionType"),4,0)
        self.cmb=QComboBox(); self.cmb.addItems(
            ["0-None","1-영역 수색","2-통로 정찰","3-이동"])
        grid.addWidget(self.cmb,4,1)

        self.bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.bb.button(QDialogButtonBox.Ok).setText("Next")       # ➜ ‘Next’ 라벨
        grid.addWidget(self.bb,5,0,1,2)
        self.bb.accepted.connect(self.goto_stage2)
        self.bb.rejected.connect(self.reject)

    # 2단계──────────────────
    def goto_stage2(self):
        self.mtype=int(self.cmb.currentIndex())
        if self.mtype==0:
            self.finish()
            return
        self.bb.setStandardButtons(QDialogButtonBox.Cancel)  # Next 없앰
        self.bb.rejected.connect(self.reject)
        # 안내
        need={1:4,2:3,3:5}[self.mtype]
        self.need=need
        self.label=QLabel(f"지도를 클릭해 점 {need}개 선택하세요 (0/{need})")
        self.layout().addWidget(self.label,6,0,1,2)
        # 통로 정찰 폭
        if self.mtype==2:
            self.layout().addWidget(QLabel("폭(m)"),7,0)
            self.spin=QDoubleSpinBox(); self.spin.setRange(1,100); self.spin.setValue(10)
            self.layout().addWidget(self.spin,7,1)
        # 브릿지 연결
        bridge.pointClicked.connect(self.collect)

    @pyqtSlot(float,float)
    def collect(self,lat,lon):
        if len(self.coords)>=self.need: return
        self.coords.append({"latitude":lat,"longitude":lon,"altitude":100})
        self.label.setText(f"지도를 클릭해 점 {self.need}개 선택하세요 ({len(self.coords)}/{self.need})")
        if len(self.coords)==self.need:
            self.finish()

    def finish(self):
        mission=make_individual_mission(self.mid)
        mission["relatedMission"]["inputMissionID"]=int(self.mpid.split('-')[-1])
        mission["individualMissionInfo"]["individualMissionType"]=self.mtype
        mission["individualMissionInfo"]["patternType"]=random.randint(0,3)
        mission["individualMissionInfo"]["autoZoomIn"]=True
        if self.mtype==1:   # 영역
            mission["individualMissionInfo"]["areaList"]=[{"isHole":False,"coordinateList":self.coords}]
        elif self.mtype==2: # 통로
            mission["individualMissionInfo"]["lineList"]=[{"width":self.spin.value(),
                                                           "coordinateList":self.coords[:2]}]
        elif self.mtype==3: # 이동
            mission["individualMissionInfo"]["coordinateList"]=self.coords
        self.missionSaved.emit(mission); self.accept()

# ───────────────────────────── 메인 GUI ─────────────────────────────
class MainGUI(QWidget):
    def __init__(self):
        super().__init__(); self.setWindowTitle("Mission Plan Generator"); self.resize(1600,900)
        self.aircraft_pool=[]; self.next_air=1
        self.missions=[]; self.next_im=1

        root=QHBoxLayout(self)
        # 지도
        self.map_frame=QFrame(); self.map_layout=QVBoxLayout(self.map_frame)
        root.addWidget(self.map_frame,2)
        self.build_map()

        # 탭
        self.tabs=QTabWidget(); root.addWidget(self.tabs,1)
        self.build_0301(); self.build_0302()

    # ───────── 지도 한 번만 세팅
    def build_map(self):
        self.write_map_html()               # folium + JS 처음 생성
        self.map_view = QWebEngineView()
        ch = QWebChannel(self.map_view.page())
        ch.registerObject("bridge", bridge) # Python ↔ JS 연결
        self.map_view.page().setWebChannel(ch)
        path = os.path.join(os.getcwd(), "map.html")   # ← 같은 코드
        self.map_view.setUrl(QUrl.fromLocalFile(path))
        self.map_layout.addWidget(self.map_view)

    # ───────── folium HTML 생성 + JS 삽입
    def write_map_html(self):
        center = [38.128774, 127.318005]
        fmap = folium.Map(location=center, zoom_start=14)
        folium.Rectangle([[38.110432,127.295620],
                          [38.147111,127.340401]],
                          color="blue", weight=1, fill=True,
                          fill_opacity=0.1).add_to(fmap)

        add_mission_shapes(fmap, self.missions)    # 임무 도형!

        path = os.path.join(os.getcwd(), "map.html")
        fmap.save(path)

        js = """
<script src="qrc:///qtwebchannel/qwebchannel.js"></script>
<script>
new QWebChannel(qt.webChannelTransport, function(ch){
  const bridge = ch.objects.bridge;
  let leaflet=null;
  for (let k in window){ if(window[k] instanceof L.Map){ leaflet=window[k]; break; } }
  if(leaflet){ leaflet.on('click', e=>bridge.sendPoint(e.latlng.lat,e.latlng.lng)); }
});
</script>"""
        with open(path, "r+", encoding="utf-8") as f:
            html = f.read()
            if "qwebchannel.js" not in html:
                f.seek(0); f.write(html.replace("</body>", js+"</body>")); f.truncate()

    # ───────── 지도 다시 그리기
    def rebuild_map(self):
        self.write_map_html()
        self.map_view.reload()

    # 0301 탭
    def build_0301(self):
        tab=QWidget(); form=QFormLayout(tab)
        self.le_mpid=QLineEdit(f"MP-{int(time.time()*1000)}")
        self.le_plid=QLineEdit("1")
        self.le_ptime=QLineEdit("1.0")
        for lab,w in (("Mission Plan ID:",self.le_mpid),
                      ("Planner ID:",self.le_plid),
                      ("Planning Time(s):",self.le_ptime)):
            form.addRow(lab,w); w.editingFinished.connect(self.refresh_0301)

        add=QPushButton("Add Aircraft"); clr=QPushButton("Clear Aircraft")
        add.clicked.connect(self.add_aircraft); clr.clicked.connect(self.clear_aircraft)
        h=QHBoxLayout(); h.addWidget(add); h.addWidget(clr); form.addRow(h)

        self.log0301=QTextEdit(); self.log0301.setReadOnly(True); form.addRow("Log:",self.log0301)
        self.tabs.addTab(tab,"0301 임무계획")
        self.refresh_0301()

    def add_aircraft(self):
        aid=self.next_air; self.next_air=1 if aid==6 else aid+1
        self.aircraft_pool.append({"aircraftID":aid,"individualMissionPlanPackageID":f"IMP-{aid:04d}"})
        self.refresh_0301(); self.refresh_air_combo()
    def clear_aircraft(self):
        self.aircraft_pool=[]; self.next_air=1; self.refresh_0301(); self.refresh_air_combo()
    def refresh_0301(self):
        now=int(time.time()*1000)
        msg={"timestamp":now,"missionPlanID":self.le_mpid.text(),
             "missionPlanTimestamp":now+100,"planningTime":float(self.le_ptime.text() or 0),
             "plannerID":int(self.le_plid.text() or 0),"aircraftList":self.aircraft_pool}
        self.log0301.setText(json.dumps(msg,ensure_ascii=False,indent=2))

    # 0302 탭
    def build_0302(self):
        tab=QWidget(); form=QFormLayout(tab)
        self.le_pkg=QLineEdit(str(random.randint(1000,9999))); form.addRow("Package ID:",self.le_pkg)
        self.cmb_air=QComboBox(); self.refresh_air_combo(); form.addRow("Aircraft ID:",self.cmb_air)

        btn_new=QPushButton("New Mission"); btn_clr=QPushButton("Clear Missions")
        btn_new.clicked.connect(self.open_wizard); btn_clr.clicked.connect(self.clear_missions)
        h=QHBoxLayout(); h.addWidget(btn_new); h.addWidget(btn_clr); form.addRow(h)

        self.log0302=QTextEdit(); self.log0302.setReadOnly(True); form.addRow("Log:",self.log0302)
        self.tabs.addTab(tab,"0302 개별 임무 계획")
        self.refresh_0302()

    def refresh_air_combo(self):
        ids=[str(a["aircraftID"]) for a in self.aircraft_pool]
        self.cmb_air.clear(); self.cmb_air.addItems(ids)

    # open_wizard -------------------------------------------
    def open_wizard(self):
        if not self.cmb_air.currentText():
            return
        self.wiz = MissionWizard(self.next_im, self.le_mpid.text(), self)
        self.wiz.missionSaved.connect(self.store_mission)
        self.wiz.show()                # non-modal


    def store_mission(self,mission):
        self.missions.append(mission); self.next_im+=1; self.refresh_0302()
    def clear_missions(self):
        self.missions=[]; self.next_im=1; self.refresh_0302()
    def refresh_0302(self):
        now=int(time.time()*1000)
        msg={"timestamp":now,"individualMissionPackageID":int(self.le_pkg.text() or 0),
             "aircraftID":int(self.cmb_air.currentText() or 0),
             "individualMissionList":self.missions}
        self.log0302.setText(json.dumps(msg,ensure_ascii=False,indent=2))
        self.rebuild_map()

    def rebuild_map(self):
        # folium 새로 생성
        center = [38.128774, 127.318005]
        fmap = folium.Map(location=center, zoom_start=14)
        folium.Rectangle([[38.110432, 127.295620],
                        [38.147111, 127.340401]],
                        color="blue", weight=1, fill=True,
                        fill_opacity=0.1).add_to(fmap)

        # 0302 임무 도형 추가
        add_mission_shapes(fmap, self.missions)

        path = os.path.join(os.getcwd(), "map.html")
        fmap.save(path)
        self.map_view.reload()   # QWebEngineView → map_view 이름으로 보관

if __name__=="__main__":
    app=QApplication(sys.argv)
    gui=MainGUI(); gui.show()
    sys.exit(app.exec_())
