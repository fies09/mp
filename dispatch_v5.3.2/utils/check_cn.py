
import re


def check_contain_chinese(check_str):
    RE = re.compile(u'[\u4e00-\u9fa5]', re.UNICODE)
    match = re.search(RE, check_str)
    if match is None:
        return False

    return True
