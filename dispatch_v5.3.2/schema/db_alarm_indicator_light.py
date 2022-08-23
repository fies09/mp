from datetime import datetime
from schema.db_base import DbBase
from schema.models import AlarmIndicatorLight
from utils.alchemy_encoder import parse_orm


class DBAlarmIndicatorLight(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = AlarmIndicatorLight

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
            if info.get('img_path'):
                result.img_path = info.get('img_path')
            result.update_time = datetime.now()

            self.commit()

        return results

    def get_alarm_data(self, robot_position_id, robot_item_id, alarm_u):
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

    def insert_data(self, info):
        data = AlarmIndicatorLight(
            robot_id=info.get("robot_id", 1),
            core_room_id=info.get("core_room_id", 1),
            robot_position_id=info.get("robot_position_id", 0),
            core_device_id=info.get("core_device_id", 0),
            robot_item_id=info.get("robot_item_id"),
            inspect_project_detail_id=info.get("inspect_project_detail_id", 0),
            level=info.get("level", 0),
            num=info.get("num", 0),
            alarm_u=info.get("alarm_u", ""),
            value=info.get("value", 0),
            type=info.get("type", 0),
            status=info.get("status", 0),
            assign=info.get("assign", 0),
            alarm_desc=info.get("alarm_desc", 0),
            img_path=info.get("img_path", ""),
            create_time=datetime.now(),
            update_time=datetime.now()
        )
        self.session.add(data)
        self.commit()
        return data.alarm_indicator_light_id


__all__ = ['DBAlarmIndicatorLight']
