from schema.db_base import DbBase
from schema.models import PowerDevice
from utils.alchemy_encoder import parse_orm


class DBPowerDevice(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = PowerDevice

    def get_device_data(self, power_device_id):
        query = self.session.query(self.obj).filter(self.obj.power_device_id == power_device_id,
                                                    self.obj.status == 0)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res
        else:
            return ""

    def get_panel_device_data(self, power_panel_id):
        query = self.session.query(self.obj).filter(self.obj.power_panel_id == power_panel_id,
                                                    self.obj.status == 0)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res
        else:
            return []


__all__ = ['DBPowerDevice']
