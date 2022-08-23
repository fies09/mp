# -*- coding: utf-8 -*-

def json_get(json, l_key, default):
    ret = json
    for k in l_key:
        if type(k) is int:
            if k < 0: return default
            if type(ret) is not list: return default
            if len(ret) <= k:  return default

        elif type(k) is str:
            if type(ret) is not dict: return default
            if k not in ret: return default
        else:
            return default

        ret = ret[k]
    return ret

