from datetime import datetime
from schema.db_base import DbBase
from schema.models import PowerAlarmRuleDetail
from utils.alchemy_encoder import parse_orm


class DBPowerAlarmRuleDetail(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = PowerAlarmRuleDetail

    def get_power_alarm_data(self, power_alarm_rule_id, item_name):
        query = self.session.query(self.obj).filter(self.obj.item_name == item_name,
                                                    self.obj.power_alarm_rule_id == power_alarm_rule_id,
                                                    self.obj.del_flag == 0,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results
        return None


__all__ = ['DBPowerAlarmRuleDetail']
