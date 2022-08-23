# from configs.log import logger
from configs.log import logger
from core.db import Session
from utils.alchemy_encoder import parse_orm
from utils.translator import dict2sql
from sqlalchemy import text, func


class DbBase(object):
    def __init__(self):
        self.session = Session
        self.obj = None

    def __del__(self):
        self.session.remove()

    def commit(self):
        try:
            self.session.commit()
        except Exception as e:
            logger.error(e)
            self.session.rollback()

    def get_all(self):
        query = self.session.query(self.obj)
        results = query.all()
        self.commit()
        return results

    def delete(self, item):
        sql = dict2sql(item)
        query = self.session.query(self.obj).filter(text(sql))
        number = query.delete(synchronize_session=False)
        self.commit()

        return number

    def judge_exist(self, info):
        try:
            sql = dict2sql(info)
            query = self.session.query(self.obj).filter(text(sql))
            results = query.all()
            self.commit()

            return results

        except Exception as e:
            # logger.error(e)
            self.session.rollback()

    def get_by_dict(self, info):
        results = parse_orm(self.judge_exist(info))
        return results

    def get_by_robot_id(self, id):
        try:
            query = self.session.query(self.obj).filter(self.obj.robot_id == id)
            results = parse_orm(query.all())
            self.commit()

            if not results:
                return None
            return results[0]
        except:
            return None

    def get_execute(self, sql_data):
        cursor = self.session.execute(sql_data)
        results = cursor.fetchall()
        if results:
            result = [dict(zip(result.keys(), result)) for result in results]
            return result
        return []
