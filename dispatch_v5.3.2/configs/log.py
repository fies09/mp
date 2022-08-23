import logging
import os
import random
import sys
import datetime

from logging import Handler
from logging.handlers import RotatingFileHandler

# 显示方式: 0（默认\）、1（高亮）、22（非粗体）、4（下划线）、24（非下划线）、 5（闪烁）、25（非闪烁）、7（反显）、27（非反显）
# 前景色:   30（黑色）、31（红色）、32（绿色）、 33（黄色）、34（蓝色）、35（洋 红）、36（青色）、37（白色）
# 背景色:   40（黑色）、41（红色）、42（绿色）、 43（黄色）、44（蓝色）、45（洋 红）、46（青色）、47（白色）

fcolor = [
    '\033[32m',
    '\033[33m',
    '\033[36m',
]


class StreamHandler(Handler):
    terminator = '\n'
    cur_color = '\033[33m'
    err_color = '\033[1;31;40m'

    def __init__(self, stream=None):
        Handler.__init__(self)
        if stream is None:
            stream = sys.stderr
        self.stream = stream

    def flush(self):
        self.acquire()
        try:
            if self.stream and hasattr(self.stream, "flush"):
                self.stream.flush()
        finally:
            self.release()

    def emit(self, record):
        try:
            msg = self.format(record)
            stream = self.stream
            if record.levelno >= logging.ERROR:
                cr = self.err_color
            else:
                cr = self._get_color(msg)

            stream.write(cr + msg + '\033[0m')
            stream.write(self.terminator)
            self.flush()

        except Exception:
            self.handleError(record)

    def _get_color(self, msg):
        if '[+]' in msg:
            num = random.randint(1, len(fcolor)) - 1
            while self.cur_color == fcolor[num]:
                num = random.randint(1, len(fcolor)) - 1
            self.cur_color = fcolor[num]

        return self.cur_color


# rpath = os.path.dirname(os.path.realpath(__file__))
# UTILS = rpath + "/"
# APP_PATH = rpath[:rpath.rfind('/')] + "/"

log_path = "/var/log/dispatch_logs/"

# todo
# prod 模式下，目录不对，还不知道为什么
# if os.environ.get('PY_ENV') == "pro":
#     log_path = "/home/udb/logs"

if os.path.exists(log_path) is False:
    os.mkdir(log_path)

# 获得当前时间
now = datetime.datetime.now()
# 转换为指定的格式
log_time = now.strftime("%Y_%m")
log_name = str(log_time)
fn = log_path + log_name + '.log'
if os.path.exists(fn) is False:
    os.mknod(fn)

logger = logging.getLogger(log_name)
logger.setLevel(level=logging.INFO)

# filehandler
#

formatter = logging.Formatter('%(asctime)s - %(filename)s - line:%(lineno)s - %(levelname)s| %(message)s')
filehandler = RotatingFileHandler(fn, maxBytes=5120000, backupCount=10, encoding="utf-8")
filehandler.setLevel(level=logging.INFO)
filehandler.setFormatter(formatter)
logger.addHandler(filehandler)

# console
#
consoleformatter = logging.Formatter('%(asctime)s - %(filename)s - line:%(lineno)s - %(levelname)s| %(message)s')
if os.environ.setdefault('PY_ENV', 'dev') == 'dev':
    consoleformatter = logging.Formatter('%(asctime)s | %(levelname)5s | %(message)s - [%(filename)s - %(lineno)s]')

console = StreamHandler()
console.setLevel(logging.INFO)
console.setFormatter(consoleformatter)
logger.addHandler(console)

# terminal
#
tf = logging.Formatter('%(asctime)s - %(filename)s - line:%(lineno)s - %(levelname)s| %(message)s')
if os.environ.setdefault('PY_ENV', 'dev') == 'dev':
    tf = logging.Formatter('\t[!] %(asctime)s | [%(processName)s : %(threadName)s] - [%(filename)s - %(lineno)s]')

terminal = StreamHandler()
terminal.setLevel(logging.ERROR)
terminal.setFormatter(tf)
logger.addHandler(terminal)

# 关闭日志输出
logging.getLogger("paramiko").setLevel(logging.ERROR)
logging.getLogger('werkzeug').setLevel(logging.ERROR)
logging.getLogger('apscheduler').setLevel(logging.ERROR)
# logging.getLogger("paramiko").setLevel(logging.INFO)
# logging.getLogger('werkzeug').setLevel(logging.INFO)
# logging.getLogger('apscheduler').setLevel(logging.INFO)

__all__ = ['logger']
