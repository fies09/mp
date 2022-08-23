from datetime import datetime

from schema.db_base import DbBase
from schema.models import CoreInventoryRecord
from utils.alchemy_encoder import parse_orm


class DBCoreInventoryRecord(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = CoreInventoryRecord

    def update(self, condition, info):
        results = self.judge_exist(condition)

        for result in results:
            if info.get('robot_id'):
                result.robot_id = info.get('robot_id')
            if info.get('inspect_project_detail_id'):
                result.inspect_project_detail_id = info.get('inspect_project_detail_id')
            if info.get('inspect_project_task_id'):
                result.inspect_project_task_id = info.get('inspect_project_task_id')
            if info.get('cabinet_count'):
                result.cabinet_count = info.get('cabinet_count')
            if info.get('device_count'):
                result.device_count = info.get('device_count')
            if info.get('device_fail_count'):
                result.device_fail_count = info.get('device_fail_count')
            if info.get('remark'):
                result.remark = info.get('remark')
            result.update_time = datetime.now()
            self.commit()

        return results

    def insert_data(self, info):
        data = CoreInventoryRecord(
            robot_id=info.get("robot_id", 1),
            inspect_project_detail_id=info.get("inspect_project_detail_id", 0),
            inspect_project_task_id=info.get("inspect_project_task_id", 0),

            cabinet_count=info.get("cabinet_count", 0),
            device_count=info.get("device_count", 0),
            device_fail_count=info.get("device_fail_count", 0),

            remark=info.get("remark", ""),
            create_time=datetime.now(),
            update_time=datetime.now()
        )
        self.session.add(data)

        self.commit()
        return data.inventory_record_id


__all__ = ['DBCoreInventoryRecord']
