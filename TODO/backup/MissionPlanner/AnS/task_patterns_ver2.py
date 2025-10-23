# 비행 패턴 정의
flight_patterns = {
    1: "OverFlight",
    2: "Offset OverFlight",
    3: "LineLoop",
    4: "LoiterAround",
    5: "LateralStandoff",
    6: "CentralStandoff"
}

# 촬영 패턴 정의
imaging_patterns = {
    1: "Back&Forth",
    2: "Spiral-Divergence",
    3: "Spiral-Convergence",
    4: "Stepwise Orthogonal Line Sweep",
    5: "LateralSwingSweep"
}

# 임무 패턴 딕셔너리 (ID 추가)
mission_patterns = {
    # 3: {
    #     "임무 패턴 명": "직하방-BF촬영",
    #     "비행 패턴": "OverFlight",
    #     "촬영 패턴": "Back&Forth"
    # },
    4: {
        "임무 패턴 명": "이격-BF촬영",
        "비행 패턴": "Offset OverFlight",
        "촬영 패턴": "Back&Forth"
    },
    # 5: {
    #     "임무 패턴 명": "구간왕복-BF촬영",
    #     "비행 패턴": "LineLoop",
    #     "촬영 패턴": "Back&Forth"
    # },
    # 6: {
    #      "임무 패턴 명": "선형반복주사-BF촬영",
    #      "비행 패턴": "SOLS-CentralStandoff",
    #      "촬영 패턴": "Back&Forth-StepwiseOrthogonalLineSweep"
    # }, ### 임무효과도 개발 필요. 
    # 7: {
    #     "임무 패턴 명": "직상공순회촬영",
    #     "비행 패턴": "LineLoop",
    #     "촬영 패턴": "Back&Forth"
    # }, ### 임무효과도 개발 필요, 비행촬영 패턴 개발 필요, 점표적정찰시 필요. 패턴모델에선 선택 안되게 보상함수
    # 8: {
    #     "임무 패턴 명": "구간중심종단-선형반복주사촬영",
    #     "비행 패턴": "CentralStandoff",
    #     "촬영 패턴": "Back&Forth-StepwiseOrthogonalLineSweep"
    # },
    # 9: {
    #     "임무 패턴 명": "구간중심종단-자동반복주사촬영",
    #     "비행 패턴": "CentralStandoff",
    #     "촬영 패턴": "LateralSwingSweep"
    # },
}
    