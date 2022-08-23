from schema.db_base import DbBase
from schema.models import PowerCabinet
from utils.alchemy_encoder import parse_orm


class DBPowerCabinet(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = PowerCabinet

    def get_room_id(self, power_cabinet_id):
        query = self.session.query(self.obj).filter(self.obj.power_cabinet_id == power_cabinet_id)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res[0].get("power_cabinet_id")
        else:
            return None


__all__ = ['DBPowerCabinet']
