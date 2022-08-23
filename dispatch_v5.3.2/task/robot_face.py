import time

# code==0为陌生人
from task.light_al import get_face_photo
from modules.move_state import movstate
from schema.db_alarm_manage import DBAlarmManage
from schema.db_inspect_project_detail import DBInspectProjectDetail
from schema.db_inspect_realtime_item_data import DBInspectRealtimeItemDatum
from task.light_al import post_voice_str
from schema.db_robot import DBRobot
from schema.db_robot_position import DBRobotPosition


def insert_face_alarm():
    db_robot_position = DBRobotPosition()

    while True:
        time.sleep(1)
        if db_robot_position.get_now_position() != 4:
            continue
        data_dict = {}
        res, face_path, face_state = get_face_photo()
        if res:
            db_inspect_realtime_item_data = DBInspectRealtimeItemDatum()
            ret = DBInspectProjectDetail().get_top1()
            if not ret:
                return None
            data_dict["inspect_project_detail_id"] = ret.get("inspect_project_detail_id")
            data_dict["robot_item_id"] = 12
            data_dict["user_id"] = 1
            robot_position = DBRobotPosition().get_now_location_all(1)
            data_dict["robot_position_id"] = robot_position.get("robot_position_id")
            data_dict["robot_position_id"] = 0
            data_dict["robot_cmd_id"] = 0
            data_dict["robot_id"] = ret.get("robot_id")
            x, y, yaw = movstate()
            data_dict["position"] = {"x": x, "y": y, "yaw": yaw}
            data_dict["img_path"] = face_path
            if len(res) > 1:
                for name in res:
                    data_dict["value"] = name
                    post_voice_str("你好{}".format(name), 1)
                    db_inspect_realtime_item_data.insert(data_dict)
            else:
                # 熟悉的人
                if face_state == 1:
                    data_dict["value"] = res[0]
                    post_voice_str("你好{}".format(res[0]), 1)
                    db_inspect_realtime_item_data.insert(data_dict)
                # 陌生人
                if face_state == 0:
                    data_dict["value"] = "陌生人"
                    post_voice_str("你没有权限,请先登记！", 1)
                    db_inspect_realtime_item_data.insert(data_dict)
                    FaceAlarm().updata_data(face_path)
        else:
            pass


class FaceAlarm(object):
    def __init__(self):
        pass

    def insert_data(self, img_path):
        data_dict = {}
        data_dict["robot_id"] = 1
        robot_position = DBRobotPosition().get_now_location_all(1)
        robot_room_id = DBRobot().get_core_room_id(1)
        if not robot_room_id:
            robot_room_id = 1
        data_dict["core_room_id"] = robot_room_id

        data_dict["robot_position_id"] = robot_position.get("robot_position_id")
        data_dict["robot_item_id"] = 12
        data_dict["inspect_project_detail_id"] = 0
        data_dict["level"] = 5
        data_dict["num"] = 1
        data_dict["value"] = "陌生人"
        data_dict["alarm_desc"] = "有陌生人进入机房"
        data_dict["img_path"] = img_path
        data_dict["status"] = 0
        data_dict["assign"] = 0
        data_dict["alarm_type"] = 4

        try:
            DBAlarmManage().insert_data(data_dict)
        except Exception as e:
            print(e)

    def updata_data(self, img_path):
        face_data = DBAlarmManage().get_face_alarm_data()
        if face_data:
            robot_position = DBRobotPosition().get_now_location_all(1)
            num = face_data.get("num")
            num += 1
            alarm_manage_id = face_data.get("alarm_manage_id")
            new_dict = {}
            new_dict["num"] = num
            new_dict["img_path"] = img_path
            new_dict["robot_position_id"] = robot_position.get("robot_position_id")
            DBAlarmManage().update({"alarm_manage_id": alarm_manage_id}, info=new_dict)
        else:
            self.insert_data(img_path)


if __name__ == '__main__':
    import rospy

    rospy.init_node("Mian_node111", disable_signals=False)
    insert_face_alarm()
