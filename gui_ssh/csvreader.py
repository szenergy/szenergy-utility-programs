class CSVReader:
    def __init__(self, filename):
        self.filename = filename
        self.data = []
        self.read()

    def read(self):
        with open(self.filename, 'r') as file:
            for actLine in file:
                actLine = actLine.strip()
                actData = actLine.split(';')
                self.data.append(actData)

    def printData(self):
        print("CSV DATA:", self.data)
