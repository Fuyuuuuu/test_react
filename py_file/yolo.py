from ultralytics import YOLO
import supervision as sv
import cv2
import numpy as np
import rospy
import matplotlib.pyplot as plt

from PIL import Image as PILImage
from PIL import ImageDraw

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge

#defect zone
x1 = 0
y1 = 0
x2 = 0
y2 = 0
#pick zone
x3 = 0
y3 = 0
x4 = 0
y4 = 0
#stay zone
x5 = 0
y5 = 0
x6 = 0
y6 = 0
#remove zone
x7 = 0
y7 = 0
x8 = 0
y8 = 0

class Count_Detection_Manager:
    def __init__(self):
        self.tracker_id_to_num = {} #Dict[int,int] #id , num
        self.num_to_tracker_id = {} #Dict[int,int] #num , id
    def update(self,detection_defect,num):
        num_tmp = num
        for tracker_id in detection_defect.tracker_id:
            self.tracker_id_to_num.setdefault(tracker_id , num_tmp)
            self.num_to_tracker_id.setdefault(num_tmp , tracker_id)
            num_tmp += 1
        return num_tmp

class Defect_Detection_Manager_fast_mode:
    def __init__(self) :
        self.tracker_id_defect = {} #Dict[int,num]={} #id , 該物品上 defect 的數量
    def update(self,detection_tracker,detection_defect):
        mask = self.defect_polygons_zone.trigger(detections=detection_tracker)
        detection_tracker=detection_tracker[mask]
        for bbox,tracker_id in zip(detection_tracker.xyxy,detection_tracker.tracker_id):
            zone = bbox + 0
            polygons_zone = sv.PolygonZone(polygon=zone,
                                        frame_resolution_wh=self.wh,
                                        triggering_position = sv.Position.CENTER)
            detection_defect = detection_defect[polygons_zone]
            num_of_defect = 0
            for class_id in detection_defect.class_id:
                if class_id == 0: #the number is the class_id of defect
                    num_of_defect+=1
            # 讓字典中儲存的是偵測到最多defect的數值
            if num_of_defect >= self.tracker_id_defect[tracker_id]:
                self.tracker_id_defect[tracker_id] = num_of_defect

class Defect_Detection_Manager_accurate_mode:
    def __init__(self) :
        self.tracker_id_defect = {} #Dict[int,num]={} #id , 該物品上 defect 的數量
    def update(self,list_of_tracker_id,list_of_detection_defect):
        for tracker_id,detection_defect in zip(list_of_tracker_id,list_of_detection_defect):
            num_of_defect = 0
            for class_id in detection_defect.class_id:
                if class_id == 0: #the number is the class_id of defect
                    num_of_defect += 1
            self.tracker_id_defect[tracker_id] = num_of_defect


class YoloDetectionNode:
    def __init__(self):
        rospy.init_node('yolo_detection_node')

        self.All_info={}
        self.All_info['Num'] = 0
        self.All_info['Pick_Ok'] = 0
        self.All_info['Pick_Defect'] = 0
        self.All_info['Stay'] = 0
        self.All_info['Remove'] = 0
        self.All_info['Stay_error'] = 0
        self.All_info['Remove_error'] = 0

        # Dict[str,int]={
        #     'Num' : int, # 此為總數在進入defect_polygons_zone中計數+1(每個物體由一個id，該id在第一次進入時要記在字典中並且跟新數值後面就可以檢查有沒有加過不會重複加到)
        #     'Pick_Ok' : int, # 此為確定沒有瑕疵的圓柱體，會在進入pick_polygons_zone中計數+1
        #     'Pick_Defect' : int, # 此為確定沒有瑕疵的圓柱體，會在進入pick_polygons_zone中計數+1

        #     'Stay' : int, # 此為確定沒有瑕疵的圓柱體，會在確定進入judge_polygons_stay_zone中計數+1
        #     'Remove' : int,  # 此為確定有瑕疵的圓柱體，會在確定進入judge_polygons_remove_zone中計數+1

        #     'Stay_error' : int, # 此為確定有瑕疵的圓柱體，會在確定進入judge_polygons_stay_zone中計數+1
        #     'Remove_error' : int,  #此為確定沒有瑕疵的圓柱體，會在確定進入judge_polygons_remove_zone中計數+1
        # }

        self.signal = rospy.Publisher('/move/signal',Int16,queue_size=5)
        self.num = rospy.Publisher('/All_info/Num',Int16,queue_size=5)
        self.pick_ok = rospy.Publisher('/All_info/Pick_Ok',Int16,queue_size=5)
        self.pick_defect = rospy.Publisher('/All_info/Pick_Defect',Int16,queue_size=5)
        self.stay = rospy.Publisher('/All_info/Stay',Int16,queue_size=5)
        self.remove = rospy.Publisher('/All_info/Remove',Int16,queue_size=5)
        self.stay_error = rospy.Publisher('/All_info/Stay_error',Int16,queue_size=5)
        self.remove_error = rospy.Publisher('/All_info/Remove_error',Int16,queue_size=5)

        #快速檢測模式、精確模式
        self.mode = 'fast' #'accurate'

        self.bridge = CvBridge()

        self.width=1280
        self.height=720
        self.wh=[self.width,self.height]

        self.defect_polygons = np.array([[x1],[y1],[x2],[y2]])
        self.pick_polygons = np.array([[x3],[y3],[x4],[y4]])
        self.judge_polygons_stay = np.array([[x5],[y5],[x6],[y6]])
        self.judge_polygons_remove = np.array([[x7],[y7],[x8],[y8]])

        self.defect_polygons_zone = sv.PolygonZone(polygon=self.defect_polygons,
                                                   frame_resolution_wh=self.wh, #待確定實際參數
                                                   triggering_position = sv.Position.CENTER)
        self.pick_polygons_zone = sv.PolygonZone(polygon=self.judge_polygons_stay,
                                                   frame_resolution_wh=self.wh,
                                                   triggering_position = sv.Position.CENTER)
        self.judge_polygons_stay_zone = sv.PolygonZone(polygon=self.judge_polygons_stay,
                                                   frame_resolution_wh=self.wh,
                                                   triggering_position = sv.Position.CENTER)
        self.judge_polygons_remove_zone = sv.PolygonZone(polygon=self.judge_polygons_remove,
                                                   frame_resolution_wh=self.wh,
                                                   triggering_position = sv.Position.CENTER)
        self.list_of_area_mask = [self.defect_polygons_zone,
                                  self.pick_polygons_zone,
                                  self.judge_polygons_stay_zone,
                                  self.judge_polygons_remove_zone]
        self.defect_model_weight_path =  './defect.pt'
        self.cylinder_model_weight_path = './cylinder.pt'

        self.model_defect = YOLO(self.defect_model_weight_path)
        self.model_cylinder = YOLO(self.cylinder_model_weight_path)

        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', ROSImage, self.process_frame_callback)

        self.tracker = sv.ByteTrack()

        #Yolo 物件追蹤 參數
        self.conf_threshold = 0.1
        self.iou_threshold = 0.1
        self.tracker_threshold = 0.1

        #Yolo 瑕疵檢測 參數
        self.conf_threshold = 0.1
        self.iou_threshold = 0.1


        #Yolo 物件追蹤 視覺化方面參數
        self.color=sv.ColorPalette.default()
        #Yolo 瑕疵檢測 視覺化方面參數


        #trace_annotator、box_annotator for 物件追蹤
        self.trace_annotator = sv.TraceAnnotator(
            color=self.color, position=sv.Position.CENTER, trace_length=100, thickness=2
        )

        self.box_annotator = sv.BoxAnnotator(thickness=2, #draw the bbox
                                      text_thickness=2,
                                      text_scale=1,
                                      color=self.color
                                      )

        self.All_detection_manager = Count_Detection_Manager()
        if self.mode == 'fast':
            self.Defect_detection_manager_fast_mode =  Defect_Detection_Manager_fast_mode()
        else:
            self.Defect_detection_manager_accurate_mode =  Defect_Detection_Manager_accurate_mode()

    #視覺化
    # 顯示 4 個區域
    # self.defect_polygons_zone self.pick_polygons_zone self.judge_polygons_stay_zone self.judge_polygons_remove_zone
    # 並且bbox只出現在那4個區域、爪子不會被框出來 (需要有遮罩)
    # 顯示是第幾個、在離開defect_polygons_zone後去顯示是否為瑕疵還是沒有瑕疵

    def annotate_frame(self, frame,detections):

        annotated_frame = frame.copy()
        #draw polyogns

        #     annotated_frame = sv.draw_polygon(
        #         annotated_frame, zone_out.polygon, COLORS.colors[i]
        #     )

        if len(self.pick_polygons_zone.tracker_id) != 0:
            print("1")
            # labels = [f"#{tracker_id}" for tracker_id in detections.tracker_id]
            # annotated_frame = self.trace_annotator.annotate(annotated_frame, detections)
            # annotated_frame = self.box_annotator.annotate(
            #     annotated_frame, detections, labels
            # )
        else:
            print()
            # labels = [f"#{tracker_id}" for tracker_id in detections.tracker_id]
            # annotated_frame = self.trace_annotator.annotate(annotated_frame, detections)
            # annotated_frame = self.box_annotator.annotate(
            #     annotated_frame, detections, labels
            # )


        # for zone_out_id, zone_out in enumerate(self.zones_out):
        #     zone_center = sv.get_polygon_center(polygon=zone_out.polygon)
        #     if zone_out_id in self.detections_manager.counts:
        #         counts = self.detections_manager.counts[zone_out_id]
        #         for i, zone_in_id in enumerate(counts):
        #             count = len(self.detections_manager.counts[zone_out_id][zone_in_id])
        #             text_anchor = sv.Point(x=zone_center.x, y=zone_center.y + 40 * i)
        #             annotated_frame = sv.draw_text(
        #                 scene=annotated_frame,
        #                 text=str(count),
        #                 text_anchor=text_anchor,
        #                 background_color=COLORS.colors[zone_in_id],
        #             )

        # return annotated_frame

    def process_defect_image(self,frame,bbox):
        pass


    def process_frame_callback(self,data):
        if self.mode == 'fast':
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            result_tracker = self.model_defect(
                frame,Verbose=False,conf=self.conf_threshold,iou=self.iou_threshold
            )[0]
            result_defect = self.model_cylinder(
                frame,Verbose=False,conf=self.conf_threshold,iou=self.iou_threshold
            )[0]

            # self.detections = sv.Detections.from_ultralytics(result)
            detection_tracker = sv.Detections.from_yolov8(result_tracker)
            detection_tracker = self.tracker.update_with_detections(detections_tracker)

            detection_defect = sv.Detections.from_yolov8(result_defect)

            '''
            if len(self.defect_polygons_zone.tracker_id) != 0:

            or

            if true in self.defect_polygons_zone.trigger(detections=detections_tracker):
            '''

            if len(self.defect_polygons_zone.tracker_id) != 0:
                num = self.All_info['Num']
                if num == 0:
                    num += 1
                new_num = self.Count_Detection_Manager.update(detection_defect,num)
                num = self.All_info['Num'] = new_num
                self.All_detection_manager.update()
                self.Defect_detection_manager_fast_mode.update(detection_tracker,detection_defect)

            if len(self.pick_polygons_zone.tracker_id) != 0:
                #此時當中的track_id 應該要只有一個因為只有一個物品
                num_of_defect = self.Defect_detection_manager_accurate_mode.tracker_id_defect[self.pick_polygons_zone.tracker_id]
                if num_of_defect == 0:
                    self.All_info['Pick_Ok'] += 1
                    # self.signal.publish(0)
                else:
                    self.All_info['Pick_Defect'] += 1
                    # move()
                    # or
                    # self.signal.publish(1)

            if len(self.judge_polygons_stay_zone.tracker_id) != 0:
                #此時當中的track_id 應該要只有一個因為只有一個物品
                num_of_defect = self.Defect_detection_manager_accurate_mode.tracker_id_defect[self.judge_polygons_stay_zone.tracker_id]
                if num_of_defect == 0:
                    self.All_info['Stay'] += 1
                else:
                    self.All_info['Stay_error'] += 1

            if len(self.judge_polygons_remove_zone.tracker_id) != 0:
                #此時當中的track_id 應該要只有一個因為只有一個物品
                num_of_defect = self.Defect_detection_manager_accurate_mode.tracker_id_defect[self.judge_polygons_remove_zone.tracker_id]
                if num_of_defect == 0:
                    self.All_info['Remove_error'] += 1
                else:
                    self.All_info['Remove'] += 1

            return self.annotate_frame(frame=frame,detections=detections_tracker)

        if self.mode == 'accurate':
            list_of_detection_defect = []
            list_of_tracker_id = []
            frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            result_tracker = self.model_defect(frame,Verbose=False,conf=self.conf_threshold,iou=self.iou_threshold)[0]

            # self.detections = sv.Detections.from_ultralytics(result)
            detections_tracker = sv.Detections.from_yolov8(result_tracker)
            detections_tracker = self.tracker.update_with_detections(detections_tracker)

            mask = self.defect_polygons_zone.trigger(detections=detection_tracker)
            detection_tracker = detection_tracker[mask]

            for bbox,tracker_id in zip(detection_tracker.xyxy,detection_tracker.tracker_id):
                frame_processed = self.process_defect_image(frame,bbox)
                result_defect = self.model_cylinder(frame_processed,Verbose=False,conf=self.conf_threshold_defect_accurate,iou=self.iou_threshold_defect_accurate)[0]
                detection_defect = sv.Detections.from_yolov8(result_defect)
                list_of_detection_defect.append(detection_defect)
                list_of_tracker_id.append(tracker_id)

            if len(self.defect_polygons_zone.tracker_id) != 0:
                num = self.All_info['Num']
                if num == 0:
                    num += 1
                new_num = self.Count_Detection_Manager.update(detection_defect,num)
                num = self.All_info['Num'] = new_num
                self.All_detection_manager.update()
                self.Defect_detection_manager_fast_mode.update(list_of_tracker_id,list_of_detection_defect)

            if len(self.pick_polygons_zone.tracker_id) != 0:
                #此時當中的track_id 應該要只有一個因為只有一個物品
                num_of_defect = self.Defect_detection_manager_accurate_mode.tracker_id_defect[self.pick_polygons_zone.tracker_id]
                if num_of_defect == 0:
                    self.All_info['Pick_Ok'] += 1
                else:
                    self.All_info['Pick_Defect'] += 1
                    # move()

            if len(self.judge_polygons_stay_zone.tracker_id) != 0:
                #此時當中的track_id 應該要只有一個因為只有一個物品
                num_of_defect = self.Defect_detection_manager_accurate_mode.tracker_id_defect[self.judge_polygons_stay_zone.tracker_id]
                if num_of_defect == 0:
                    self.All_info['Stay'] += 1
                else:
                    self.All_info['Stay_error'] += 1

            if len(self.judge_polygons_remove_zone.tracker_id) != 0:
                #此時當中的track_id 應該要只有一個因為只有一個物品
                num_of_defect = self.Defect_detection_manager_accurate_mode.tracker_id_defect[self.judge_polygons_remove_zone.tracker_id]
                if num_of_defect == 0:
                    self.All_info['Remove_error'] += 1
                else:
                    self.All_info['Remove'] += 1
            #publisher
            self.num.publish(self.All_info['Num'])
            self.pick_ok.publish(self.All_info['Pick_Ok'])
            self.pick_defect.publish(self.All_info['Pick_Defect'])
            self.stay.publish(self.All_info['Stay'])
            self.remove.publish(self.All_info['Remove'])
            self.stay_error.publish(self.All_info['Stay_error'])
            self.remove_error.publish(self.All_info['Remove_error'])

            return self.annotate_frame(frame=frame,detections=detections_tracker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = YoloDetectionNode()
    node.run()