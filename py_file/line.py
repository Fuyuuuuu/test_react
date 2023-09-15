from fastapi import FastAPI, HTTPException, Request, BackgroundTasks
from linebot import LineBotApi, WebhookHandler
from linebot.exceptions import InvalidSignatureError
from linebot.models import (MessageEvent, TextMessage, TextSendMessage,
                            PostbackEvent, URIAction, RichMenu, RichMenuArea,
                            RichMenuBounds, RichMenuSize)
from pymongo import MongoClient
from linebot.models import PostbackAction
import time

app = FastAPI()

# LineBot API Configurations
CHANNEL_ACCESS_TOKEN = "YOUR_CHANNEL_ACCESS_TOKEN"
line_bot_api = LineBotApi(CHANNEL_ACCESS_TOKEN)
handler = WebhookHandler("YOUR_CHANNEL_SECRET")
# MongoDB Configurations
client = MongoClient("mongodb://localhost:27017/")
db = client["your_database_name"]
status_info_collection = db["status_info"]
# User ID for sending proactive messages
USER_ID = 'YOUR_USER_ID'
IMAGE_PATH = "path_to_your_background_image.jpg"


@app.post("/webhook/")
async def webhook_endpoint(request: Request):
    # Get request body and headers
    body = await request.body()
    headers = request.headers

    # Get X-Line-Signature header value
    signature = headers['X-Line-Signature']

    # Handle webhook body
    try:
        handler.handle(body.decode(), signature)
    except InvalidSignatureError:
        raise HTTPException(status_code=400, detail="Invalid request")
    return 'OK'

# Handle postback actions from the Rich Menu
from linebot.models import PostbackEvent, TextSendMessage

@handler.add(PostbackEvent)
def handle_postback(event: PostbackEvent):
    data = event.postback.data

    if data == "action_for_current_status":
        # 獲取相關資料或URL
        status_message = "Current status: ... | URL: [Your Homepage URL1]"
        line_bot_api.reply_message(event.reply_token, TextSendMessage(text=status_message))

    elif data == "action_for_pickup_info":
        info_message = "Pickup information: ... | URL: [Your Homepage URL2]"
        line_bot_api.reply_message(event.reply_token, TextSendMessage(text=info_message))

    elif data == "action_for_monitor_image":
        image_message = "Monitor Image: ... | URL: [Your Homepage URL3]"
        line_bot_api.reply_message(event.reply_token, TextSendMessage(text=image_message))

    elif data == "action_for_data_page":
        data_message = "Data page info: ... | URL: [Your Data Page URL]"
        line_bot_api.reply_message(event.reply_token, TextSendMessage(text=data_message))

    elif data == "action_for_line_qrcode":
        qrcode_message = "Line QR Code: ... | URL: [Your Line Page URL]"
        line_bot_api.reply_message(event.reply_token, TextSendMessage(text=qrcode_message))

    else:
        line_bot_api.reply_message(event.reply_token, TextSendMessage(text="Unknown action."))

rich_menu_to_create = RichMenu(
    size=RichMenuSize(width=2500, height=843),
    selected=True,
    name="Custom Menu",
    chat_bar_text="Open Menu",
    areas=[
        # 上面三個按鈕
        RichMenuArea(
            bounds=RichMenuBounds(x=0, y=0, width=833, height=421),
            action=PostbackAction(label="Current Status", data="action_for_current_status")
        ),
        RichMenuArea(
            bounds=RichMenuBounds(x=833, y=0, width=833, height=421),
            action=PostbackAction(label="Pickup Info", data="action_for_pickup_info")
        ),
        RichMenuArea(
            bounds=RichMenuBounds(x=1666, y=0, width=834, height=421),
            action=PostbackAction(label="Monitor Image", data="action_for_monitor_image")
        ),
        # 下面兩個按鈕
        RichMenuArea(
            bounds=RichMenuBounds(x=0, y=421, width=1250, height=422),
            action=PostbackAction(label="Data Page", data="action_for_data_page")
        ),
        RichMenuArea(
            bounds=RichMenuBounds(x=1250, y=421, width=1250, height=422),
            action=PostbackAction(label="Line QRCode", data="action_for_line_qrcode")
        ),
    ]
)



rich_menu_id = line_bot_api.create_rich_menu(rich_menu=rich_menu_to_create)
print(f"Rich Menu ID: {rich_menu_id}")

# Upload Image to the RichMenu
IMAGE_PATH = "path_to_your_rich_menu_image.jpg"
with open(IMAGE_PATH, 'rb') as f:
    line_bot_api.set_rich_menu_image(rich_menu_id, "image/jpeg", f)

# For proactive messages
def check_status_and_send_message():
    while True:
        latest_status = status_info_collection.find_one(sort=[("timestamp", -1)])
        if latest_status and latest_status['program_status'] in ["start", "end"]:
            message = f"Program has {latest_status['program_status']} at {latest_status['timestamp']}"
            line_bot_api.push_message(USER_ID, TextSendMessage(text=message))
        time.sleep(60)

@app.post("/start_checking/")
async def start_checking(background_tasks: BackgroundTasks):
    background_tasks.add_task(check_status_and_send_message)
    return {"status": "Background task started"}

