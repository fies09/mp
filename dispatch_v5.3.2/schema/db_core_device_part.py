from schema.db_base import DbBase
from schema.models import CoreDevicePart
from utils.alchemy_encoder import parse_orm


class DBCoreDevicePart(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = CoreDevicePart

    def get_core_device_data(self, robot_position_id, start_u):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == robot_position_id,
                                                    self.obj.start_u == start_u,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None

    def get_core_device_part(self, robot_position_id, start_u, end_u):
        query = self.session.query(self.obj).filter(self.obj.robot_position_id == robot_position_id,
                                                    self.obj.start_u == start_u,
                                                    self.obj.end_u == end_u,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return {}


__all__ = ['DBCoreDevicePart']
