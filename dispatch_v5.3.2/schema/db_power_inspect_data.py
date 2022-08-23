from datetime import datetime

from schema.db_base import DbBase
from schema.models import PowerInspectDatum
from utils.alchemy_encoder import parse_orm


class DBPowerInspectDatum(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = PowerInspectDatum

    def insert(self, info):
        data = PowerInspectDatum(
            inspect_project_detail_id=info.get("inspect_project_detail_id", 1),
            inspect_project_task_id=info.get("inspect_project_task_id", 1),
            user_id=info.get("user_id", 1),
            robot_id=info.get("robot_id", 1),
            power_robot_item_id=info.get("power_robot_item_id", 0),
            robot_position_id=info.get("robot_position_id", ""),
            power_cabinet_id=info.get("power_cabinet_id", ""),
            power_device_id=info.get("power_device_id", ""),
            value=info.get("value", ""),
            img_path=info.get("img_path"),
            type=info.get("type"),
            status=info.get("status", 0),
            del_flag=info.get("del_flag", 0),
            create_time=datetime.now(),
            create_by=info.get("create_by", 1),
            update_by=info.get("update_by"),
            remark=info.get("remark", ""),
        )
        self.session.add(data)
        self.commit()
        return data.power_inspect_data_id

    def get_item_data(self, power_robot_item_id):
        query = self.session.query(self.obj).filter(self.obj.power_robot_item_id == power_robot_item_id)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res
        else:
            return []

    def update(self, uuid_list, info):
        if uuid_list:
            for uuid in uuid_list:
                condition = {"power_inspect_data_id": uuid}
                results = self.judge_exist(condition)
                for result in results:
                    if info.get('img_path'):
                        result.img_path = info.get('img_path')
                    result.update_time = datetime.now()
            self.commit()


__all__ = ['DBPowerInspectDatum']
