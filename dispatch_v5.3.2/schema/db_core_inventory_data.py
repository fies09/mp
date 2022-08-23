from datetime import datetime

from schema.db_base import DbBase
from schema.models import CoreInventoryDatum
from utils.alchemy_encoder import parse_orm
from sqlalchemy import func, distinct, distinct


class DBCoreInventoryDatum(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = CoreInventoryDatum

    def insert_data(self, info):
        data = CoreInventoryDatum(
            robot_path_id=info.get("robot_path_id", 1),
            inventory_id=info.get("inventory_id", 0),
            inventory_record_id=info.get("inventory_record_id", 1),
            robot_id=info.get("robot_id", 1),
            robot_position_id=info.get("robot_position_id", 0),
            inspect_project_detail_id=info.get("inspect_project_detail_id", 0),
            inventory_status=info.get("inventory_status", 0),
            status=info.get("status", 0),
            remark=info.get("remark", ""),
            create_time=datetime.now(),
            update_time=datetime.now()
        )
        self.session.add(data)

        self.commit()

    def get_rfid_data(self, inspect_project_detail_id):
        query = self.session.query(self.obj).filter(self.obj.inspect_project_detail_id == inspect_project_detail_id). \
            filter(self.obj.status == 1)
        result = query.all()

        self.commit()
        if result:
            results = parse_orm(result)
            return results
        return result


__all__ = ['DBCoreInventoryDatum']
