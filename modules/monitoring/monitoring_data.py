import threading

class MonitoringData:
    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(MonitoringData, cls).__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self.data = {}
        self._initialized = True

    def get_data(self, key, default=None):
        return self.data.get(key, default)

    def set_data(self, key, value):
        self.data[key] = value

    def get_all_data(self):
        return self.data.copy()

# Singleton instance
monitoring_data_instance = MonitoringData()
