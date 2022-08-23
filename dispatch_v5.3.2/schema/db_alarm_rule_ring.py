from datetime import datetime
from schema.db_base import DbBase
from schema.models import AlarmRuleRing
from utils.alchemy_encoder import parse_orm


class DBAlarmRuleRing(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = AlarmRuleRing

    def get_alarm_data(self, item_name):
        query = self.session.query(self.obj).filter(self.obj.item_name == item_name,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None



__all__ = ['DBAlarmRuleRing']
