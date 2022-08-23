# -*- coding: utf-8 -*-
import json
import datetime


def convert_to_dict(obj):
    '''把Object对象转换成Dict对象'''
    dict = {}
    dict.update(obj.__dict__)
    return dict


def convert_to_dicts(objs):
    '''把对象列表转换为字典列表'''
    obj_arr = []

    for o in objs:
        # 把Object对象转换成Dict对象
        dict = {}
        dict.update(o.__dict__)
        obj_arr.append(dict)

    return obj_arr


def class_to_dict(obj):
    '''把对象(支持单个对象、list、set)转换成字典'''
    is_list = obj.__class__ == [].__class__
    is_set = obj.__class__ == set().__class__

    if is_list or is_set:
        obj_arr = []
        for o in obj:
            # 把Object对象转换成Dict对象
            dict = {}
            dict.update(o.__dict__)
            obj_arr.append(dict)
        return obj_arr
    else:
        dict = {}
        dict.update(obj.__dict__)
        return dict


class Dict2Obj(dict):
    def __init__(self, *args, **kwargs):
        super(Dict2Obj, self).__init__(*args, **kwargs)

    def __getattr__(self, key):
        value = self[key]
        if isinstance(value, dict):
            value = Dict2Obj(value)
        return value


class CJsonEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, datetime.datetime):
            return obj.strftime('%Y-%m-%d %H:%M:%S')
        elif isinstance(obj, datetime.date):
            return obj.strftime("%Y-%m-%d")
        else:
            return json.JSONEncoder.default(self, obj)


# 将字典转换成json
def dict_to_json(po):
    jsonstr = json.dumps(po, ensure_ascii=False, cls=CJsonEncoder)
    return jsonstr


# 将json转换成字典
def json_to_dict(jsonstr):
    if isinstance(jsonstr, bytes):
        jsonstr = jsonstr.decode("utf-8")
    d = json.loads(jsonstr)
    return d
