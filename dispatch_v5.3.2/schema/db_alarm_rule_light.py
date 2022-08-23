from datetime import datetime
from schema.db_base import DbBase
from schema.models import AlarmRuleLight
from utils.alchemy_encoder import parse_orm


class DBAlarmRuleLight(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = AlarmRuleLight

    def get_light_rule_data(self, rule_id, name, item_name):
        query = self.session.query(self.obj).filter(self.obj.rule_id == rule_id,
                                                    self.obj.name == name,
                                                    self.obj.item_name == item_name,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None


__all__ = ['DBAlarmRuleLight']
