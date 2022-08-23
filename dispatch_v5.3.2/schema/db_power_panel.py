from schema.db_base import DbBase
from schema.models import PowerPanel
from utils.alchemy_encoder import parse_orm


class DBPowerPanel(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = PowerPanel

    def get_panel_data(self, power_cabinet_id):
        query = self.session.query(self.obj).filter(self.obj.power_cabinet_id == power_cabinet_id,
                                                    self.obj.status == 0)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res
        else:
            return []


__all__ = ['DBPowerPanel']
