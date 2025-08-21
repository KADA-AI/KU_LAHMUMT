# receive/database.py
class ReceivedDatabase(object):
    _instance = None
    _MESSAGE_CODES = [
        "0101",
        "0201",
        "0202",
        "0203",
        "0301",
        "0302",
        "0303",
        "0304",
        "0401",
        "0402",
        "0501",
        "0701",
        "0702",
        "0801",
        "0802",
        "0803",
        "0804",
        "0805",
        "0806",
        "0901",
        "0902",
        "0903"
    ]

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
            # 각 메시지 코드에 대한 초기 속성을 리스트로 생성
            for code in cls._MESSAGE_CODES:
                setattr(cls._instance, f"received_{code}", [])
        return cls._instance

    # 0101 메시지 (시스템 운용모드)
    def get_received_0101(self):
        return self.received_0101

    def set_received_0101(self, value):
        self.received_0101 = value

    # 0201 메시지 (협업기저임무 계획)
    def get_received_0201(self):
        return self.received_0201

    def set_received_0201(self, value):
        self.received_0201 = value

    # 0202 메시지 (선행임무정보)
    def get_received_0202(self):
        return self.received_0202

    def set_received_0202(self, value):
        self.received_0202 = value

    # 0203 메시지 (비행참조정보)
    def get_received_0203(self):
        return self.received_0203

    def set_received_0203(self, value):
        self.received_0203 = value

    # 0301 메시지 (MissionPlan)
    def get_received_0301(self):
        return self.received_0301

    def set_received_0301(self, value):
        self.received_0301 = value

    # 0302 메시지 (IndividualMissionPlan)
    def get_received_0302(self):
        return self.received_0302

    def set_received_0302(self, value):
        self.received_0302 = value

    # 0303 메시지 
    def get_received_0303(self):
        return self.received_0303

    def set_received_0303(self, value):
        self.received_0303 = value

    # 0304 메시지 
    def get_received_0304(self):
        return self.received_0304

    def set_received_0304(self, value):
        self.received_0304 = value

    # 0401 메시지 (유무인기 상태정보)
    def get_received_0401(self):
        return self.received_0401

    def set_received_0401(self, value):
        self.received_0401 = value

    # 0402 메시지 (전장상황인지정보)
    def get_received_0402(self):
        return self.received_0402

    def set_received_0402(self, value):
        self.received_0402 = value

    # 0501 메시지 
    def get_received_0501(self):
        return self.received_0501

    def set_received_0501(self, value):
        self.received_0501 = value

    # 0701 메시지 
    def get_received_0701(self):
        return self.received_0701

    def set_received_0701(self, value):
        self.received_0701 = value

    # 0702 메시지 (의사결정 결과)
    def get_received_0702(self):
        return self.received_0702

    def set_received_0702(self, value):
        self.received_0702 = value

    # 0801 메시지 (운용자 임무재계획 명령)
    def get_received_0801(self):
        return self.received_0801

    def set_received_0801(self, value):
        self.received_0801 = value

    # 0802 메시지 (강제명령)
    def get_received_0802(self):
        return self.received_0802

    def set_received_0802(self, value):
        self.received_0802 = value

    # 0803 메시지 (다음 협업기저임무 수행 명령)
    def get_received_0803(self):
        return self.received_0803

    def set_received_0803(self, value):
        self.received_0803 = value

    # 0804 메시지 (협업기저임무 재수행 명령)
    def get_received_0804(self):
        return self.received_0804

    def set_received_0804(self, value):
        self.received_0804 = value

    # 0805 메시지 (임무종료 명령)
    def get_received_0805(self):
        return self.received_0805

    def set_received_0805(self, value):
        self.received_0805 = value

    # 0806 메시지 (SW종료 명령)
    def get_received_0806(self):
        return self.received_0806

    def set_received_0806(self, value):
        self.received_0806 = value

    # 0901 메시지 
    def get_received_0901(self):
        return self.received_0901

    def set_received_0901(self, value):
        self.received_0901 = value

    # 0902 메시지 
    def get_received_0902(self):
        return self.received_0902

    def set_received_0902(self, value):
        self.received_0902 = value

    # 0903 메시지 
    def get_received_0903(self):
        return self.received_0903

    def set_received_0903(self, value):
        self.received_0903 = value


# 싱글톤 인스턴스
received_db = ReceivedDatabase()
