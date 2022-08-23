import time
from datetime import datetime

from sqlalchemy import desc
from schema.db_base import DbBase
from schema.models import RobotVoice
from utils.alchemy_encoder import parse_orm


class DBRobotVoice(DbBase):
    def __init__(self):
        super().__init__()
        self.obj = RobotVoice

    def get_by_name(self, name):
        query = self.session.query(self.obj).filter(self.obj.name == name)
        result = query.all()
        self.commit()
        if result:
            results = parse_orm(result)
            content = results[0]
            return content
        else:
            return ""

    def get_play(self,name):
        query = self.session.query(self.obj).filter(self.obj.name == name)
        result = query.all()
        self.commit()

        if result:
            results = parse_orm(result)
            content = results[0].get("is_play")
            return content
        else:
            return ""

__all__ = ['DBRobotVoice']
