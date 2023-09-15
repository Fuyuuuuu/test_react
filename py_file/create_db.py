from pymongo import MongoClient
from bson.json_util import dumps

# 定義pick_info的JSON Schema
pick_info_schema = {
    "$jsonSchema": {
        "bsonType": "object",
        "required": ["num", "pick_ok", "pick_defect", "stay", "remove", "stay_error", "remove_error", "timestamp"],
        "properties": {
            "num": {
                "bsonType": "int"
            },
            "pick_ok": {
                "bsonType": "int"
            },
            "pick_defect": {
                "bsonType": "int"
            },
            "stay": {
                "bsonType": "int"
            },
            "remove": {
                "bsonType": "int"
            },
            "stay_error": {
                "bsonType": "int"
            },
            "remove_error": {
                "bsonType": "int"
            },
            "task_id": {
                "bsonType": "string",
                "description": "可選的task_id"
            },
            "timestamp": {
                "bsonType": "date"
            }
        }
    }
}

# 定義status_info的JSON Schema
status_info_schema = {
    "$jsonSchema": {
        "bsonType": "object",
        "required": ["program_status", "timestamp"],
        "properties": {
            "program_status": {
                "bsonType": "bool",
                "description": "start表示為true，end表示為false"
            },
            "timestamp": {
                "bsonType": "date"
            }
        }
    }
}

# 假設的MongoDB連接 (在實際環境中，您需要連接到真實的MongoDB伺服器)
client = MongoClient('mongodb://localhost:27017/')
db = client.picking_database

# 使用JSON Schema創建collections
db.create_collection("pick_info", validator=pick_info_schema)
db.create_collection("status_info", validator=status_info_schema)
