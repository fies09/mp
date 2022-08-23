from datetime import datetime
from schema.db_base import DbBase
from schema.models import PowerAlarm
from utils.alchemy_encoder import parse_orm


class DBPowerAlarm(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = PowerAlarm

    def insert(self, info):
        data = PowerAlarm(
            inspect_project_detail_id=info.get("inspect_project_detail_id", 1),
            #user_id=info.get("user_id", 1),
            robot_id=info.get("robot_id", 1),
            core_room_id=info.get("core_room_id", 1),
            power_robot_item_id=info.get("power_robot_item_id", 0),
            robot_position_id=info.get("robot_position_id", ""),
            power_device_id=info.get("power_device_id", ""),
            level=info.get("level", 5),
            value=info.get("value", ""),
            num=info.get("num", ""),
            img_path=info.get("img_path"),
            assign=info.get("assign", 0),
            alarm_desc=info.get("alarm_desc", 0),
            create_time=datetime.now(),
            update_time=datetime.now(),
            create_by=info.get("create_by", 1),
            update_by=info.get("update_by"),
            remark=info.get("remark", ""),
        )
        self.session.add(data)
        self.commit()

    def update(self, condition, info):
        results = self.judge_exist(condition)
        for result in results:
            if info.get('num'):
                result.num = info.get('num')
            if info.get('common_num'):
                result.common_num = info.get('common_num')
            if info.get('undo_status'):
                result.undo_status = info.get('undo_status')
            if info.get('inspect_project_detail_id'):
                result.inspect_project_detail_id = info.get('inspect_project_detail_id')
            if info.get('value'):
                result.value = info.get('value')
            if info.get('alarm_desc'):
                result.alarm_desc = info.get('alarm_desc')
            if info.get('img_path'):
                result.img_path = info.get('img_path')
            result.update_time = datetime.now()

            self.commit()

        return results

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


__all__ = ['DBPowerAlarm']
