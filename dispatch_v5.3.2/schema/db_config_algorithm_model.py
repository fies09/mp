from schema.db_base import DbBase
from schema.models import ConfigAlgorithmModel
from utils.alchemy_encoder import parse_orm


class DBConfigAlgorithmModel(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = ConfigAlgorithmModel

    def get_model_data(self, config_algorithm_model_id):
        query = self.session.query(self.obj).filter(self.obj.config_algorithm_model_id == config_algorithm_model_id)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res[0]
        else:
            return ""

    def get_name_data(self, name):
        query = self.session.query(self.obj).filter(self.obj.name == name)
        self.commit()

        result = query.all()
        if result:
            res = parse_orm(result)
            return res[0]
        else:
            return ""


__all__ = ['DBConfigAlgorithmModel']
