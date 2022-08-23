# -*- coding: utf-8 -*-

import time
import os
import sys

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, scoped_session

from configs import db_mysql
from configs.log import logger
from urllib import parse

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)


def make_engine():
    while True:
        try:
            # uri = "mysql+pymysql://{user}:{pwd}@{host}:{port}/{database}".format(**db_mysql)
            uri = "mysql+pymysql://{user}:{pwd}@{host}:{port}/{database}".format(user=db_mysql["user"],
                                                                                 pwd=parse.quote_plus(db_mysql["pwd"]),
                                                                                 host=db_mysql["host"],
                                                                                 port=db_mysql["port"],
                                                                                 database=db_mysql["database"])
            new_engine = create_engine(uri, echo=False, encoding='utf-8',
                                       pool_recycle=3600,
                                       pool_size=300,
                                       pool_pre_ping=True,
                                       max_overflow=200,
                                       )
            if new_engine:
                return new_engine
            else:
                raise
        except Exception as e:
            logger.error(e)
            time.sleep(30)
        finally:
            new_engine.dispose()

engine = make_engine()
session_factory = sessionmaker(bind=engine, expire_on_commit=False)
Session = scoped_session(session_factory)

__all__ = ['Session']
