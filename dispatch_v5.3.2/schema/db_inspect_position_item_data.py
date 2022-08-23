from datetime import datetime

from schema.db_base import DbBase
from schema.models import InspectPositionItemDatum


class DBInspectPositionItemDatum(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = InspectPositionItemDatum

    def insert(self, info):
        data = InspectPositionItemDatum(
            inspect_project_detail_id=info.get("inspect_project_detail_id", 1),
            inspect_project_task_id=info.get("inspect_project_task_id", 1),
            user_id=info.get("user_id", 1),
            robot_id=info.get("robot_id", 1),
            robot_item_id=info.get("robot_item_id", 0),
            robot_position_id=info.get("robot_position_id", ""),
            core_cabinet_id=info.get("core_cabinet_id", ""),
            core_device_id=info.get("core_device_id", ""),
            server_u=info.get("server_u", 0),
            value=info.get("value", ""),
            img_path=info.get("img_path"),
            video_path=info.get("video_path"),
            sound_path=info.get("sound_path"),
            type=info.get("type", 1),
            status=info.get("status", 0),
            create_time=datetime.now(),
            create_by=info.get("create_by", 1),
            update_by=info.get("update_by"),
            remark=info.get("remark", ""),
            # crop_path=info.get("crop_path", ""),
            power_cabinet_id=info.get("power_cabinet_id", 0),
            play_desc=info.get("play_desc", ""),
            play_status=info.get("play_status", 0),
            alarm_rule_id=info.get("alarm_rule_id", 0),
            device_part_id=info.get("device_part_id", 0),
            robot_device_id=info.get("robot_device_id", 0)
        )
        self.session.add(data)
        self.commit()
        return data.position_item_data_id

    def update(self, condition, info):
        results = self.judge_exist(condition)

        for result in results:

            if info.get('status'):
                result.status = info.get('status')
            else:
                result.update_time = datetime.now()
            self.commit()

        return results



__all__ = ['DBInspectPositionItemDatum']

