from .monitoring_data import monitoring_data_instance

class MonitoringLogicHandler:
    def __init__(self):
        self.data_store = monitoring_data_instance

    def process_data(self):
        # Example logic: get some data from the singleton and process it
        # This is where the core logic of the monitoring module will go.
        all_data = self.data_store.get_all_data()
        print("Processing data:", all_data)
        # Add more complex processing logic here
        processed_result = {"status": "processed", "data_keys": list(all_data.keys())}
        self.data_store.set_data("last_processed_result", processed_result)
        return processed_result
