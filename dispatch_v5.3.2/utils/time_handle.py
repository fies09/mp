import calendar
import time
import datetime
import pandas as pd

from dateutil import tz
from interval import Interval


def last_day():
    d = datetime.datetime.now()

    oneday = datetime.timedelta(days=1)
    day = d - oneday
    date_from = datetime.datetime(day.year, day.month, day.day, 0, 0, 0)
    date_to = datetime.datetime(day.year, day.month, day.day, 23, 59, 59)

    return date_from, date_to


def last_week():
    d = datetime.datetime.now()
    dayscount = datetime.timedelta(days=d.isoweekday())
    dayto = d - dayscount

    sixdays = datetime.timedelta(days=6)
    dayfrom = dayto - sixdays
    date_from = datetime.datetime(dayfrom.year, dayfrom.month, dayfrom.day, 0, 0, 0)
    date_to = datetime.datetime(dayto.year, dayto.month, dayto.day + 1, 0, 0, 0)

    return date_from, date_to


def last_month():
    d = datetime.datetime.now()

    dayscount = datetime.timedelta(days=d.day)
    dayto = d - dayscount
    date_from = datetime.datetime(dayto.year, dayto.month, 1, 0, 0, 0)
    date_to = datetime.datetime(dayto.year, dayto.month, dayto.day + 1, 0, 0, 0)

    return date_from, date_to


# 返回某一周的开始、结束时间，周一为一周的开始
# return: datetime
def get_week_time(year, week_num):
    cal = datetime.date(year, 1, 4)
    cal += datetime.timedelta(7 - cal.weekday())

    dayfrom = datetime.timedelta(weeks=week_num - 1)
    end = cal + dayfrom
    start = end - datetime.timedelta(days=7)

    date_from = datetime.datetime(start.year, start.month, start.day, 0, 0, 0)
    date_to = datetime.datetime(end.year, end.month, end.day, 0, 0, 0)

    return date_from, date_to


# 返回月份的开始、结束时间
# return: datetime
def get_month_time(year, month):
    firstdayweekday, month_range = calendar.monthrange(year, month)

    start = datetime.date(year=year, month=month, day=1)
    end = datetime.date(year=year, month=month, day=month_range)

    date_from = datetime.datetime(start.year, start.month, start.day, 0, 0, 0)
    date_to = datetime.datetime(end.year, end.month, end.day, 0, 0, 0)

    return date_from, date_to


def datetime2timestamp(_datetime):
    if isinstance(_datetime, float) or isinstance(_datetime, int):
        return _datetime
    return time.mktime(_datetime.timetuple())


def utc2timestamp(_utc):
    dt_obj = datetime.datetime.strptime(_utc, '%Y-%m-%dT%H:%M:%S.%fZ')

    from_zone = tz.gettz('UTC')
    to_zone = tz.gettz('CST')
    dt_obj = dt_obj.replace(tzinfo=from_zone)
    dt_obj = dt_obj.astimezone(to_zone)

    return datetime2timestamp(dt_obj)


def isostr2utc8str(isostr):
    utc = datetime.datetime.strptime(isostr, '%Y-%m-%dT%H:%M:%S.%fZ')
    utc8time = utc.replace(tzinfo=tz.tzutc()).astimezone(tz.tzlocal()).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    return utc8time


def parse_interval_tab_string(time_unit, interval_time):
    if time_unit == 0:
        return {'hours': interval_time}
    elif time_unit == 1:
        return {'days': interval_time}
    elif time_unit == 2:
        return {'months': interval_time}


def split_time_ranges(from_time, to_time, frequency):
    from_time, to_time = pd.to_datetime(from_time), pd.to_datetime(to_time)
    time_range = list(pd.date_range(from_time, to_time, freq='%sS' % frequency))
    if to_time not in time_range:
        time_range.append(to_time)
    time_range = [item.strftime("%Y-%m-%d %H:%M:%S") for item in time_range]
    time_ranges = []
    for item in time_range:
        f_time = item
        t_time = (datetime.datetime.strptime(item, "%Y-%m-%d %H:%M:%S") + datetime.timedelta(seconds=frequency))
        if t_time >= to_time:
            t_time = to_time.strftime("%Y-%m-%d %H:%M:%S")
            time_ranges.append([f_time, t_time])
            break
        time_ranges.append([f_time, t_time.strftime("%Y-%m-%d %H:%M:%S")])
    return time_ranges


def alive_time(from_time, to_time, time_data):
    target_time = Interval(from_time, to_time)
    test_time = Interval(time_data, time_data)
    if test_time in target_time:
        return True
    else:
        return False


if __name__ == '__main__':
    get_month_time(2019, 10)
    get_week_time(2019, 43)
    last_week()
