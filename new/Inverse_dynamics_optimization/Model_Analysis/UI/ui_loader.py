# ui_loader.py

import json
import csv

# =========================
# Load / Save
# =========================
def load_node_names(path):

    names = []

    with open(path, newline='', encoding='utf-8-sig') as f:

        reader = csv.DictReader(f)

        for row in reader:
            names.append(row["name"])

    return names

def load_rod_names(path):

    names = []

    with open(path, newline='', encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        for row in reader:
            names.append(row["name"])

    return names

def load_json(path):
    if path.exists():
        return json.load(open(path, "r", encoding="utf-8"))
    return {}

def save_json(data,path):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

# ===================================(for static)
class ConfigLoader:

    def __init__(self, path):
        with open(path, "r", encoding="utf-8") as f:
            self.cfg = json.load(f)

    def get(self):
        return self.cfg

    def static(self):
        return self.cfg.get("static", {})

