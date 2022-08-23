from datetime import datetime

from schema.db_base import DbBase
from schema.models import AlarmDeviceCheck
from utils.alchemy_encoder import parse_orm


class DBAlarmDeviceCheck(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = AlarmDeviceCheck

    def insert_data(self, info):
        data = AlarmDeviceCheck(
            robot_id=info.get("robot_id", 1),
            inspect_project_detail_id=info.get("inspect_project_detail_id", 0),
            core_room_id=info.get("core_room_id", 0),
            core_cabinet_id=info.get("core_cabinet_id", 0),
            robot_item_id=info.get("robot_item_id", 0),
            core_device_id=info.get("core_device_id", 0),
            num=info.get("num", 0),
            assign=info.get("assign", 0),
            del_flag=0,
            alarm_desc=info.get("alarm_desc", ""),
            create_time=datetime.now(),
            update_time=datetime.now()
        )
        self.session.add(data)

        self.commit()
        return data.device_check_id

    def update(self, condition, info):
        results = self.judge_exist(condition)
        for result in results:
            if info.get('num'):
                result.num = info.get('num')
            if info.get('inspect_project_detail_id'):
                result.inspect_project_detail_id = info.get('inspect_project_detail_id')
            result.update_time = datetime.now()

            self.commit()

        return results

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


__all__ = ['DBAlarmDeviceCheck']
