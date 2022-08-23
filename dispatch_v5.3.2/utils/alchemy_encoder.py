# coding=utf-8
import datetime
import json

from sqlalchemy.ext.declarative import DeclarativeMeta


def parse_orm(obj):
    result = []
    if type(obj) == list:
        for i in obj:
            result.append(reflect(i))

    elif type(obj) == dict:
        result.append(reflect(obj))

    else:
        result.append(reflect(obj))

    return result


def reflect(obj):
    if isinstance(obj.__class__, DeclarativeMeta):
        fields = {}
        for field in [x for x in dir(obj) if not x.startswith('_') and x != 'metadata']:
            data = obj.__getattribute__(field)
            try:
                json.dumps(data)
                fields[field] = data

            except TypeError:
                if isinstance(data, datetime.datetime):
                    fields[field] = data.isoformat()
                elif isinstance(data, datetime.date):
                    fields[field] = data.isoformat()
                elif isinstance(data, datetime.timedelta):
                    fields[field] = (datetime.datetime.min + data).time().isoformat()
                else:
                    fields[field] = None
        return fields
    return json.JSONEncoder.default(obj)
