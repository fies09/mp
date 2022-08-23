from datetime import datetime
from schema.db_base import DbBase
from schema.models import AlarmRuleIndcamera
from utils.alchemy_encoder import parse_orm


class DBAlarmRuleIndcamera(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = AlarmRuleIndcamera

    def get_light_rule_data(self, core_device_id):
        query = self.session.query(self.obj).filter(self.obj.core_device_id == core_device_id,
                                                    self.obj.del_flag == 0,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None


__all__ = ['DBAlarmRuleIndcamera']
