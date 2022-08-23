import os
import sys
from datetime import datetime

from .singleton import singleton


@singleton
class Path(object):
    def __init__(self):
        self.rpath = os.path.dirname(os.path.realpath(__file__))
        self.UTILS = self.rpath + "/"
        self.APP_PATH = self.rpath[:self.rpath.rfind('/')] + "/"

        self.test_data = self.APP_PATH + 'testdata/'
        self.congfig = self.APP_PATH + 'congfig/'
        self.modules = self.APP_PATH + 'modules/'
        self.thermal = self.APP_PATH + 'thermal_path/' + datetime.now().strftime("%Y_%m_%d") + "/"
        self.photo = self.APP_PATH + "data/" + 'photo_path/'
        self.temp = self.APP_PATH + "data/" + 'temp_path/'

        # self.voice_lib = self.APP_PATH + "data/" + 'voice_lib/'
        self.voice_lib = self.APP_PATH + 'voice_lib/'
        self.face = self.APP_PATH + 'face/'
        self.video_lib = self.APP_PATH + "data/" + 'video_path/'
        self.face_path = self.APP_PATH + "data/" + 'face_path/'
        self.test_photo = self.APP_PATH + 'test_pkg/'
        self.gongye_img = self.APP_PATH + 'gongye_img/'
        self.industry = self.APP_PATH + "data/" + 'industry_path/'
        self.pointer = self.APP_PATH + "data/" + 'pointer_path/'

    @staticmethod
    def get_app_path():
        return os.getcwd()

    @staticmethod
    def get_app_name():
        name = os.path.basename(sys.argv[0])
        return '.'.join(name.split('.')[:-1])

    @staticmethod
    def get_script_path():
        return os.path.dirname(os.path.abspath(__file__))

    @staticmethod
    def get_script_name():
        name = os.path.basename(__file__)
        return '.'.join(name.split('.')[:-1])
