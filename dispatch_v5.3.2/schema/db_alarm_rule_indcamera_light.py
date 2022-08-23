from datetime import datetime
from schema.db_base import DbBase
from schema.models import AlarmRuleIndcameraLight
from utils.alchemy_encoder import parse_orm


class DBAlarmRuleIndcameraLight(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = AlarmRuleIndcameraLight

    def get_light_rule_data(self, alarm_rule_indcamera_id, item_name):
        query = self.session.query(self.obj).filter(self.obj.alarm_rule_indcamera_id == alarm_rule_indcamera_id,
                                                    self.obj.item_name == item_name,
                                                    self.obj.del_flag == 0,
                                                    self.obj.status == 0)
        result = query.all()

        self.commit()

        if result:
            results = parse_orm(result)
            return results[0]
        return None


__all__ = ['DBAlarmRuleIndcameraLight']
