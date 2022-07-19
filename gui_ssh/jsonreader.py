import json

class JSONReader:
    def __init__(self, filename):
        self.filename = filename
        self.data = []
        self.readData(self.filename)
    
    def readData(self, filename):
        with open(filename, "r") as f:
            self.data = json.load(f)