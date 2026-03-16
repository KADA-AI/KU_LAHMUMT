# modules/common/receive/database.py

class ReceivedDatabase(object):
    _instance = None
    _MESSAGE_CODES = [
        # 0000 Response(ACK/NACK 등 공통 응답)
        "0000",
        "0001",
        # 010x 시스템 운용/상태
        "0101", "0102", "0103",
        # 020x 임무 입력/선행/참조
        "0201", "0202", "0203",
        # 030x 계획류
        "0301", "0302", "0303", "0304", "0305",
        # 040x 상황/상태
        "0401", "0402",
        # 050x 임무 상태/진전(세분)
        "0501", "0502", "0503", "0504"
        # 060x 기타(예: 링크/시스템 등)
        "0601",
        # 070x 옵션/결과
        "0701", "0702",
        # 080x 명령
        "0801", "0802", "0803", "0804", "0805", "0806",
        # 090x 운용자 요청
        "0901", "0902", "0903",
    ]

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
            # 각 메시지 코드에 대한 초기 속성을 리스트로 생성
            for code in cls._MESSAGE_CODES:
                setattr(cls._instance, f"received_{code}", [])
        return cls._instance

    # 범용 getter/setter (신규)
    def get_received(self, code: str):
        attr = f"received_{code}"
        return getattr(self, attr)

    def set_received(self, code: str, value):
        attr = f"received_{code}"
        setattr(self, attr, value)

    # === 아래는 기존 호환용 개별 getter/setter들 ===
    # 0000
    def get_received_0000(self): return self.received_0000
    def set_received_0000(self, value): self.received_0000 = value
    def get_received_0001(self): return self.received_0001
    def set_received_0001(self, value): self.received_0001 = value

    # 0101/0102/0103
    def get_received_0101(self): return self.received_0101
    def set_received_0101(self, value): self.received_0101 = value
    def get_received_0102(self): return self.received_0102
    def set_received_0102(self, value): self.received_0102 = value
    def get_received_0103(self): return self.received_0103
    def set_received_0103(self, value): self.received_0103 = value

    # 0201/0202/0203
    def get_received_0201(self): return self.received_0201
    def set_received_0201(self, value): self.received_0201 = value
    def get_received_0202(self): return self.received_0202
    def set_received_0202(self, value): self.received_0202 = value
    def get_received_0203(self): return self.received_0203
    def set_received_0203(self, value): self.received_0203 = value

    # 0301/0302/0303/0304/0305
    def get_received_0301(self): return self.received_0301
    def set_received_0301(self, value): self.received_0301 = value
    def get_received_0302(self): return self.received_0302
    def set_received_0302(self, value): self.received_0302 = value
    def get_received_0303(self): return self.received_0303
    def set_received_0303(self, value): self.received_0303 = value
    def get_received_0304(self): return self.received_0304
    def set_received_0304(self, value): self.received_0304 = value
    def get_received_0305(self): return self.received_0305
    def set_received_0305(self, value): self.received_0305 = value

    # 0401/0402
    def get_received_0401(self): return self.received_0401
    def set_received_0401(self, value): self.received_0401 = value
    def get_received_0402(self): return self.received_0402
    def set_received_0402(self, value): self.received_0402 = value

    # 0501/0502/0503
    def get_received_0501(self): return self.received_0501
    def set_received_0501(self, value): self.received_0501 = value
    def get_received_0502(self): return self.received_0502
    def set_received_0502(self, value): self.received_0502 = value
    def get_received_0503(self): return self.received_0503
    def set_received_0503(self, value): self.received_0503 = value
    def get_received_0504(self): return self.received_0504
    def set_received_0504(self, value): self.received_0504 = value

    # 0601
    def get_received_0601(self): return self.received_0601
    def set_received_0601(self, value): self.received_0601 = value

    # 0701/0702
    def get_received_0701(self): return self.received_0701
    def set_received_0701(self, value): self.received_0701 = value
    def get_received_0702(self): return self.received_0702
    def set_received_0702(self, value): self.received_0702 = value

    # 0801~0806
    def get_received_0801(self): return self.received_0801
    def set_received_0801(self, value): self.received_0801 = value
    def get_received_0802(self): return self.received_0802
    def set_received_0802(self, value): self.received_0802 = value
    def get_received_0803(self): return self.received_0803
    def set_received_0803(self, value): self.received_0803 = value
    def get_received_0804(self): return self.received_0804
    def set_received_0804(self, value): self.received_0804 = value
    def get_received_0805(self): return self.received_0805
    def set_received_0805(self, value): self.received_0805 = value
    def get_received_0806(self): return self.received_0806
    def set_received_0806(self, value): self.received_0806 = value

    # 0901/0902/0903
    def get_received_0901(self): return self.received_0901
    def set_received_0901(self, value): self.received_0901 = value
    def get_received_0902(self): return self.received_0902
    def set_received_0902(self, value): self.received_0902 = value
    def get_received_0903(self): return self.received_0903
    def set_received_0903(self, value): self.received_0903 = value


# 싱글톤 인스턴스
received_db = ReceivedDatabase()
