import base64


class DataProcessor:
    def __init__(self):
        self.decoded_data = None

    def decode_data(self, encoded_data):
        decoded_bytes = base64.b64decode(encoded_data)
        return decoded_bytes.decode("utf-8")

    def calculate(self, data):
        return len(data)

    def process_data(self):
        encoded_data = self.data_generator()
        decoded_data = self.decode_data(encoded_data)
        result = self.calculate(decoded_data)
        self.save_data(decoded_data)
        self.send_data(decoded_data)
        return result

    def save_data(self, data):
        self.decoded_data = data

    def send_data(self, data):
        self.client.publish(self.mqtt_topic, data)
