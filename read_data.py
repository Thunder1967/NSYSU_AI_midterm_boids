import json
import initial
from datetime import datetime

def read_Setting(file_name = "setting_default.json"):
    if ".json" in file_name:
        try:
            with open(f"data/{file_name}", "r") as f:
                Setting = json.load(f)
                if Setting.get("verify","")=="Setting" :return Setting,file_name
        except FileNotFoundError or json.JSONDecodeError:
            pass
    print(f'Can not find "{file_name}" or the format error. Create new one.')
    initial.create_default()
    file_name = initial.create_default()
    with open(f"data/{file_name}", "r") as f:
        Setting = json.load(f)
    return Setting,file_name

def read_Setting_no_default(file_name):
    Setting={}
    if ".json" in file_name:
        try:
            with open(f"data/{file_name}", "r") as f:
                Setting = json.load(f)
        except FileNotFoundError or json.JSONDecodeError:
            pass
    if Setting.get("verify","")=="Setting": return Setting
    else : return None

def read_OLD(file_name = None):
    if file_name is None or ".json" not in file_name:
        return None,"No load any file"
    else:
        try:
            with open(f"data/{file_name}", "r") as f:
                data = json.load(f)
                if data.get("verify","")=="OLD": return data,file_name
        except FileNotFoundError or json.JSONDecodeError:
            pass
        print(f'Can not find "{file_name}" or the format error.')
        return None,"No load any file"

def save_OLD(data,file_name = ""):
    if ".json" not in file_name: file_name = f"data/simulation_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json"
    with open(f"data/{file_name}", "w") as f:
            json.dump(data, f)

def save_Setting(data,file_name = ""):
    if ".json" not in file_name: file_name = f"data/setting_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json"
    with open(f"data/{file_name}", "w") as f:
            json.dump(data, f)

def open_new_record(file_name=""):
    if ".json" not in file_name: file_name = f"data/record/record_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.json"
    with open(file_name, "w") as f:
        f.write("[")
    return file_name

def record_simulation(data,file_name,comma=","):
    json_line = json.dumps(data, ensure_ascii=False)
    with open(file_name, "a") as f:
        f.write(f'{comma}\n{json_line}')

def close_record(file_name):
    with open(file_name, "a") as f:
        f.write("\n]")