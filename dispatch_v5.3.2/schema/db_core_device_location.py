from datetime import datetime
from schema.db_base import DbBase
from schema.models import CoreDeviceLocation
from utils.alchemy_encoder import parse_orm


class DBCoreDeviceLocation(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = CoreDeviceLocation

    def get_core_device_id(self, core_cabinet_id):
        query = self.session.query(self.obj).filter(self.obj.core_cabinet_id == core_cabinet_id,
                                                    self.obj.oper_type == 0,
                                                    self.obj.status == 0). \
            order_by(self.obj.start_u)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results
        return None

    def get_start_u(self, core_device_id):
        query = self.session.query(self.obj).filter(self.obj.core_device_id == core_device_id,
                                                    self.obj.oper_type == 0,
                                                    self.obj.status == 0). \
            order_by(self.obj.start_u)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_by_cabinet_id(self, core_device_id):
        query = self.session.query(self.obj).filter(self.obj.core_device_id == core_device_id,
                                                    self.obj.oper_type == 0,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_core_cabinet_id(self, core_device_id):
        query = self.session.query(self.obj).filter(self.obj.core_device_id == core_device_id)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            results = res[0].get("core_cabinet_id")
            return results
        else:
            return ""


__all__ = ['DBCoreDeviceLocation']
