from schema.db_base import DbBase
from schema.models import CoreDevice
from utils.alchemy_encoder import parse_orm


class DBCoreDevice(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = CoreDevice

    def get_core_device_data(self, core_device_id):
        query = self.session.query(self.obj).filter(self.obj.core_device_id == core_device_id,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None


__all__ = ['DBCoreDevice']
