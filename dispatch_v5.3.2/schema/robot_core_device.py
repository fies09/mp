from schema.db_base import DbBase
from schema.models import CoreDevice
from utils.alchemy_encoder import parse_orm


class DBCoreDevice(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = CoreDevice

    def get_device_id(self, core_device_id):
        query = self.session.query(self.obj).filter(self.obj.core_device_id == core_device_id)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            results = res[0]
            return results
        else:
            return ""


__all__ = ['DBCoreDevice']
