from datetime import datetime
from schema.db_base import DbBase
from schema.models import AlarmRealtimeDatum
from utils.alchemy_encoder import parse_orm


class DBAlarmRealtimeDatum(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = AlarmRealtimeDatum

    def update(self, condition, info):
        results = self.judge_exist(condition)

        for result in results:
            if info.get('num'):
                result.num = info.get('num')
            if info.get('position'):
                result.position = info.get('position')
            if info.get('img_path'):
                result.img_path = info.get('img_path')
            result.update_time = datetime.now()

            self.commit()

        return results

    def get_alarm_data(self):
        query = self.session.query(self.obj).filter(self.obj.value == "陌生人",
                                                    self.obj.status == 0,
                                                    self.obj.undo_status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def insert_data(self, info):
        data = AlarmRealtimeDatum(
            robot_id=info.get("robot_id", 1),
            core_room_id=info.get("core_room_id", 0),
            robot_position_id=info.get("robot_position_id", 0),
            robot_item_id=info.get("robot_item_id"),
            inspect_project_detail_id=info.get("inspect_project_detail_id", 0),
            level=info.get("level", datetime.now()),
            num=info.get("num", 0),
            value=info.get("value", 0),
            alarm_desc=info.get("alarm_desc", "0"),
            position=info.get("position"),
            img_path=info.get("img_path"),
            status=info.get("status", 0),
            assign=info.get("assign", 0),
            create_time=datetime.now(),
            update_time=datetime.now()
        )
        self.session.add(data)
        self.commit()



__all__ = ['DBAlarmRealtimeDatum']
