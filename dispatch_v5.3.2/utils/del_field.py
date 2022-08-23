# -*- coding: utf-8 -*-


def del_key(res, key_name):
    for ret in res:
        del ret[key_name]

    return res
