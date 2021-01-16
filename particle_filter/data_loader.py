import os
import re
import csv
import pandas as pd


class DataLoader:
    def __init__(self, data_dir, identifier):
        self.data_dir = data_dir
        self.identifier = identifier
        self.data_file_regex = r".*_(%s)-(.*).csv" % (str(identifier))
        self.df = None

        self.load()

    def load(self):
        for filename in os.listdir(self.data_dir):
            match = self.match_data_file(filename)
            if not match:
                continue
            identifier = match.group(1)
            topic = match.group(2)

            if self.identifier != identifier:
                continue

            path = os.path.join(self.data_dir, filename)
            self.parse_data_file(path)

    def match_data_file(self, filename):
        return re.search(self.data_file_regex, filename)

    def parse_data_file(self, path):
        print(path)
        self.df = pd.read_csv(path)

