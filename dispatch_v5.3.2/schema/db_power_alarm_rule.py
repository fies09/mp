from datetime import datetime
from schema.db_base import DbBase
from schema.models import PowerAlarmRule
from utils.alchemy_encoder import parse_orm


class DBPowerAlarmRule(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = PowerAlarmRule

    def get_power_alarm_rule(self, power_alarm_rule_id):
        query = self.session.query(self.obj).filter(self.obj.power_alarm_rule_id == power_alarm_rule_id,
                                                    self.obj.del_flag == 0,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None


__all__ = ['DBPowerAlarmRule']
