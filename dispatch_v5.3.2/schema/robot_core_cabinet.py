from schema.db_base import DbBase
from schema.models import CoreCabinet
from utils.alchemy_encoder import parse_orm


class DBCoreCabinet(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = CoreCabinet

    def get_cabinet_name(self, core_cabinet_id):
        query = self.session.query(self.obj).filter(self.obj.core_cabinet_id == core_cabinet_id)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            results = res[0]
            return results
        else:
            return ""


__all__ = ['DBCoreCabinet']
