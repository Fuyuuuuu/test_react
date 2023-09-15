# main.py
from fastapi import FastAPI
from pymongo import MongoClient

app = FastAPI()

# 建立到 MongoDB 的連接
client = MongoClient("mongodb://localhost:27017/")
db = client["your_database_name"]  # 更改為您的數據庫名稱

@app.get("/latest_data")
async def get_latest_data():
    data = db.pick_info.find().sort("_id", -1).limit(1)
    return data[0] if data else {}

@app.get("/control_status")
async def get_control_status():
    # 此處可以根據您的需求獲取和回傳控制狀態
    return {"status": "running"}
