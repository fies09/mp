from datetime import datetime

from schema.db_base import DbBase
from schema.models import InspectRealtimeItemDatum


class DBInspectRealtimeItemDatum(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = InspectRealtimeItemDatum

    def insert(self, info):
        data = InspectRealtimeItemDatum(
            user_id=info.get("user_id", 1),
            robot_cmd_id=info.get("robot_cmd_id", ""),
            robot_id=info.get("robot_id", 1),
            robot_item_id=info.get("robot_item_id", 0),
            robot_position_id=info.get("robot_position_id", ""),
            inspect_project_detail_id=info.get("inspect_project_detail_id", ""),
            position=info.get("position", ""),
            value=info.get("value", ""),
            img_path=info.get("img_path"),
            create_time=datetime.now(),
            update_time=datetime.now(),
            create_by=info.get("create_by", ),
            update_by=info.get("update_by"),
            remark=info.get("remark", ""),
        )

        self.session.add(data)
        self.commit()


__all__ = ['DBInspectRealtimeItemDatum']
