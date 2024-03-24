import rclpy
from math import cos, pi
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from visualization_msgs.msg import MarkerArray
from cv_msg.msg import Object, ObjectList, ClassList


class FakeTracker(Node):

    def __init__(self):
        super().__init__('fake_tracker')

        self.publisher = self.create_publisher(ClassList, 'tracker_out', 10)
        self.subscription = self.create_subscription(
            ClassList,
            '/localization_out',
            self.listener_callback,
            10)
        self.prev_msg = ""
        self.class_dict = {  
                        0: [],
                        1: [],  # Buildings, Works
                        2: [],
                        3: [],
                        4: [],
                        5: [],
                        6: [],
                        7: [],
                        8: [],
                        9: [],
                        10: [],  # Vehicles, Works
                        11: [],
                        12: [],  # TrafficSigns, 
                        13: [],
                        14: [],
                        15: [],
                        16: [],
                        17: [],
                        18: [],  # TrafficLights, Works
                        19: [],
                        20: [],
                        21: [],
                        22: []
                        }
        self.prev_dict = self.class_dict

    def track_obj_from_msg(self, cls_id, obj):
        new_id = self.new_id_pointer
        used_id = []

        cur_x = obj.img_x + obj.img_w/2
        cur_y = obj.img_y + obj.img_h/2

        prev_objects = self.prev_dict[cls_id]
        
        for prev_obj in prev_objects:
            prev_x = prev_obj.img_x + prev_obj.img_w/2
            prev_y = prev_obj.img_y + prev_obj.img_h/2
            used_id.append(prev_obj.id)

            if prev_obj.id == new_id:
                new_id += 1

            if abs(cur_x - prev_x) < 20 and abs(cur_y - prev_y) < 5: #####################################
                 obj.id = prev_obj.id
                 return obj   

        while new_id in used_id:
            new_id += 1
    
        obj.id = new_id
        self.new_id_pointer += 1
        return obj
            
         
    def listener_callback(self, msg):
        if self.prev_msg == "":
             self.prev_msg = msg

        classes = msg
        for cls in classes.class_objs: # Для каждого класса объектов в листе классов
            self.new_id_pointer = 1
            for obj in cls.objects:  # Для каждого объекта в листе объектов одного класса
                obj = self.track_obj_from_msg(cls.id, obj)
            self.class_dict[cls.id] = cls.objects
        
        self.prev_dict = self.class_dict
        # print(self.prev_dict)
        self.publisher.publish(classes)

    
def main(args=None):
    rclpy.init(args=args)

    fake_tracker = FakeTracker()

    rclpy.spin(fake_tracker)

    fake_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

