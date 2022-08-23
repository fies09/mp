from schema.db_base import DbBase
from schema.models import PowerCameraParam
from utils.alchemy_encoder import parse_orm


class DBPowerCameraParam(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = PowerCameraParam

    def get_power_data(self, power_device_id):
        query = self.session.query(self.obj).filter(self.obj.power_device_id == power_device_id, self.obj.status == 0)
        self.commit()
        result = query.all()
        if result:
            res = parse_orm(result)
            return res
        else:
            return ""

    def get_power_param_data(self, robot_algorithm_id, item_id):
        query = self.session.query(self.obj).filter(self.obj.robot_algorithm_id == robot_algorithm_id,
                                                    self.obj.power_robot_item_id == item_id,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            res = parse_orm(result)
            return res[0]
        return None


__all__ = ['DBPowerCameraParam']
