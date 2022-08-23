from datetime import datetime
from schema.db_base import DbBase
from schema.models import AlarmManage
from utils.alchemy_encoder import parse_orm


class DBAlarmManage(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = AlarmManage

    def update(self, condition, info):
        results = self.judge_exist(condition)

        for result in results:
            if info.get('num'):
                result.num = info.get('num')
            if info.get('common_num'):
                result.common_num = info.get('common_num')
            if info.get('undo_status'):
                result.undo_status = info.get('undo_status')
            if info.get('img_path'):
                result.img_path = info.get('img_path')
            if info.get('inspect_project_detail_id'):
                result.inspect_project_detail_id = info.get('inspect_project_detail_id')
            if info.get('alarm_desc'):
                result.alarm_desc = info.get('alarm_desc')
            if info.get('value'):
                result.value = info.get('value')
            if info.get('robot_position_id'):
                result.robot_position_id = info.get('robot_position_id')
            result.update_time = datetime.now()

            self.commit()

        return results

    def get_alarm_light_data(self, robot_position_id, robot_item_id, alarm_u):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == robot_position_id,
                                                    self.obj.robot_item_id == robot_item_id,
                                                    self.obj.alarm_u == alarm_u,
                                                    self.obj.status == 0,
                                                    self.obj.undo_status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_alarm_data(self, robot_position_id, robot_item_id):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == robot_position_id,
                                                    self.obj.robot_item_id == robot_item_id,
                                                    self.obj.status == 0,
                                                    self.obj.undo_status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_by_core_device_id(self, core_device_id):
        query = self.session.query(self.obj).filter(self.obj.core_device_id == core_device_id,
                                                    self.obj.del_flag == 0,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()
        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_power_alarm_data(self, robot_position_id, power_robot_item_id, power_device_id):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == robot_position_id,
                                                    self.obj.power_robot_item_id == power_robot_item_id,
                                                    self.obj.power_device_id == power_device_id,
                                                    self.obj.del_flag == 0,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def insert_data(self, info):
        data = AlarmManage(
            robot_id=info.get("robot_id", 1),
            core_room_id=info.get("core_room_id", 0),
            inspect_project_detail_id=info.get("inspect_project_detail_id", 0),
            inspect_project_task_id=info.get("inspect_project_task_id", 0),

            robot_position_id=info.get("robot_position_id", 0),
            core_cabinet_id=info.get("core_cabinet_id",0),
            core_device_id=info.get("core_device_id"),
            core_device_part_id=info.get("core_device_part_id"),
            alarm_u=info.get("alarm_u"),
            robot_item_id=info.get("robot_item_id"),
            robot_device_id=info.get("robot_device_id", 0),
            power_cabinet_id=info.get("power_cabinet_id"),
            power_device_id=info.get("power_device_id"),
            power_robot_item_id=info.get("power_robot_item_id"),
            alarm_type=info.get("alarm_type", 6),
            level=info.get("level", 5),
            num=info.get("num", 1),
            value=info.get("value", 0),
            alarm_desc=info.get("alarm_desc"),
            status=info.get("status", 0),

            assign=info.get("assign", 0),
            img_path=info.get("img_path"),

            undo_status=info.get("undo_status", 0),
            inspect_time=datetime.now(),
            create_time=datetime.now(),
            update_time=datetime.now(),

        )
        self.session.add(data)

        self.commit()
        return data.alarm_manage_id

    def get_face_alarm_data(self):
        query = self.session.query(self.obj).filter(self.obj.alarm_type == 8,
                                                    self.obj.status == 0,
                                                    self.obj.undo_status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None


__all__ = ['DBAlarmManage']
