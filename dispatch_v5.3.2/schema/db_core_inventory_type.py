from schema.db_base import DbBase
from schema.models import CoreInventoryType
from utils.alchemy_encoder import parse_orm
from sqlalchemy import func, distinct, distinct


class DBCoreInventoryType(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = CoreInventoryType

    def get_by_rfid(self, rfid):
        query = self.session.query(self.obj).filter(self.obj.rfid == rfid)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_by_device_id(self, core_device_id):
        query = self.session.query(self.obj).filter(self.obj.core_device_id == core_device_id)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_rfid_all(self, core_room_id):
        query = self.session.query(self.obj.rfid).filter(self.obj.core_room_id == core_room_id). \
            filter(self.obj.inventory_type == 1)
        result = query.all()

        self.commit()

        if result:
            result_list = []
            for i in result:
                result_list.append(i[0])
            return result_list
        return None

    def get_rfid_device_all(self):
        query = self.session.query(func.count(distinct(self.obj.core_device_id)))
        result = query.all()

        self.commit()

        if result:
            results = result[0][0]
            return results
        return None

    def get_rfid_cabinet_all(self):
        query = self.session.query(func.count(distinct(self.obj.core_cabinet_id)))
        result = query.all()
        self.commit()

        if result:
            results = result[0][0]
            return results
        return None

    def get_list(self, device_ids:list, **kwargs):
        items = self.session.query(self.obj).filter(self.obj.del_flag == 0)
        if len(device_ids) > 0:
            items = items.filter(self.obj.core_device_id.in_(device_ids))

        if kwargs["inventory_type"] > 0:
            items = items.filter(self.obj.inventory_type == kwargs["inventory_type"])

        if kwargs["core_cabinet_id"] > 0:
            items = items.filter(self.obj.core_cabinet_id == kwargs["core_cabinet_id"])

        rows = items.order_by(self.obj.create_time.desc()).all()

        total = items.count()

        return rows, total

__all__ = ['DBCoreInventoryType']
