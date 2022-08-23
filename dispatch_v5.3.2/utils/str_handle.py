# -*- coding: utf-8 -*-

def get_sub_string(str, start, end):
    start_i = str.find(start)
    end_i = str.find(end, start_i + len(start))
    if start_i == -1 or end_i == -1:
        return ''

    sub_str = str[start_i + len(start):end_i]
    return sub_str


# 从右侧开始查找字符串，并返回查找到的字符串右侧的字符串
def get_rstring(info, substr):
    idx = info.rindex(substr) + len(substr)

    return info[idx:]
