# -*- coding: utf-8 -*-
# Created by dysec on 2019/7/4 上午11:58


def singleton(cls, *args, **kw):
    instances = {}

    def _singleton():
        if cls not in instances:
            instances[cls] = cls(*args, **kw)
        return instances[cls]

    return _singleton
