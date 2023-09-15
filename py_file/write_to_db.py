import rospy
from std_msgs.msg import Int16
from pymongo import MongoClient


class Write_to_db():
    def __init__(self):
        self.num = rospy.Subscriber('/All_info/Num' , Int16 , self.num_callback)
        self.pick_ok = rospy.Subscriber('/All_info/Num' , Int16 , self.pick_ok_callback)
        self.pick_defect = rospy.Publisher('/All_info/Pick_Defect',Int16,self.pick_defect_callback)
        self.stay = rospy.Publisher('/All_info/Stay',Int16,self.stay_callback)
        self.remove = rospy.Publisher('/All_info/Remove',Int16,self.remove_callback)
        self.stay_error = rospy.Publisher('/All_info/Stay_error',Int16,self.stay_error_callback)
        self.remove_error = rospy.Publisher('/All_info/Remove_error',Int16,self.remove_error_callback)
    def num_callback(self,msg):
        pass
    def pick_ok_callback(self,msg):
        pass
    def pick_defect_callback(self,msg):
        pass
    def stay_callback(self,msg):
        pass
    def remove_callback(self,msg):
        pass
    def stay_error_callback(self,msg):
        pass
    def remove_error_callback(self,msg):
        pass





