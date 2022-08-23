import base64
import json
import os
import requests
from configs.log import logger
from configs import pointer_ips


# 调用AGX算法接口
def post_agx_light(photo_name, start_imgRows, end_imgRows,
                   start_imgCols, end_imgCols, show_data, type=0):
    image = []
    if os.path.exists(photo_name):
        with open(photo_name, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')
            image.append(img)
    input_get = {'picPathList': None,  # 图片路径 *
                 'picDataList': image,  # 原始图片数据 *
                 'picType': type,
                 'rois':
                     [
                         {
                             'startCol': start_imgCols,
                             'startRow': start_imgRows,
                             'endCol': end_imgCols,
                             'endRow': end_imgRows,
                             'legend': show_data,
                             'id': 1
                         }
                     ],  # 检测框
                 'isBlend': 0,  # 是否执行图片融合
                 'bNum': 1,
                 'isCrop': 1}

    # 访问服务
    try:
        ret = requests.post("http://192.168.10.134:8087/algoritmic_server/light", data=json.dumps(input_get))
        if ret.status_code == 200:
            data = json.loads(ret.text)
            logger.info(data)
            light_detect_result = data['results'][0]
            light_path = data.get("resPath")
            narrow_path = data.get("narrowPath")
            return light_detect_result, light_path, narrow_path
        else:
            return None, None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问算法接口错误")
        return None, None, None


def post_agx_light_industry(photo_name, rois, show_data, type=0):
    image = []
    if os.path.exists(photo_name):
        # photo_name = "/opt/moss_robot/lib/dispatch_ips_web/data_pkg/industry_path/2021_12_29/1.jpg"
        with open(photo_name, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')
            image.append(img)
    new_rois = list()
    for r in rois:
        dict_data = {}
        dict_data["startCol"] = int(r["start_imgCols"])
        dict_data["startRow"] = int(r["start_imgRows"])
        dict_data["endCol"] = int(r["end_imgCols"])
        dict_data["endRow"] = int(r["end_imgRows"])
        dict_data["legend"] = r["show_data"]
        dict_data["id"] = r["id"]
        new_rois.append(dict_data)
    # logger.info(new_rois)

    input_get = {
        'picPathList': photo_name,  # 图片路径 *
        'picDataList': image,  # 原始图片数据 *
        'picType': type,
        'rois': new_rois,  # 检测框
        'isBlend': 0,  # 是否执行图片融合
        'isCrop': 1,
        'bNum': 1
    }
    # 访问服务
    try:
        ret = requests.post("http://192.168.10.134:8087/algoritmic_server/light", data=json.dumps(input_get))
        if ret.status_code == 200:
            # data = eval(ret.content.decode('utf-8'))
            data = json.loads(ret.text)
            logger.info("算法识别结果：{}".format(data))
            narrow_path = data.get("narrowPath", None)
            data["narrowPath"] = narrow_path
            if not narrow_path:
                data["narrowPath"] = data["resPath"]
            return data["results"], data["narrowPath"]
        else:
            return None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问算法接口错误")
        return None, None
# 数字仪表读数服务接口
def post_agx_digital(photo_name, u_params_list, type=1):
    if os.path.exists(photo_name):
        with open(photo_name, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')
            # image.append(img)
            input_get = {
                'picPathStr': None,  # 图片路径 *
                'picData': img,  # 原始图片数据 *
                'picType': type,  # 图片类型（四倍1，工业2，web3）
                "isPreview": 1,
                'rois': u_params_list
            }
    try:
        ret = requests.post(
            "http://{}:{}/algoritmic_server/digit".format(pointer_ips.get("host"), pointer_ips.get("port")),
            data=json.dumps(input_get))
        # logger.info(input_get)
        if ret.status_code == 200:
            data = json.loads(ret.content.decode('utf-8'))
            logger.info("数字仪表算法返回结果")
            logger.info(data)
            light_detect_result = data.get("results")  # 解析结果
            # light_path = data.get("resPath")  # 图片存储路径
            narrow_path = data.get("narrowPath", None)  # 缩略路径
            if not narrow_path:
                light_path = data.get("resPath")  # 图片存储路径
                narrow_path = light_path
            # 绘图服务参数获取
            plot_data = data.get("plots", None)
            return light_detect_result, narrow_path, plot_data
        else:
            logger.error("访问算法接口没有成功！")
            return None, None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问算法接口错误")
        return None, None, None


# 开关服务接口
def post_agx_switch(photo_name, u_params_list, type=1):
    try:
        rois = u_params_list
    except Exception as e:
        logger.error(e)
        logger.error("传入开关算法数据正确")
        return None, None, None
    # 临时图片
    # photo_name = "/opt/moss_robot/lib/dispatch_analysis/service_api/test/1.jpg"
    if os.path.exists(photo_name):
        with open(photo_name, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')
            input_get = {
                'picPathStr': None,  # 图片路径 *
                'picData': img,  # 原始图片数据 *
                'picType': type,  # 图片类型（四倍1，工业2，web3）
                'isPreview': True,  # 是否保存本地图片
                'rois': rois
            }
    try:
        ret = requests.post(
            "http://{}:{}/algoritmic_server/rswitch".format(pointer_ips.get("host"), pointer_ips.get("port")),
            data=json.dumps(input_get))
        if ret.status_code == 200:
            data = json.loads(ret.content.decode('utf-8'))
            logger.info("开关服务算法返回结果")
            logger.info(data)
            light_detect_result = data.get("results", None)
            # light_path = data.get("resPath")  # 图片存储路径
            narrow_path = data.get("narrowPath", None)  # 缩略路径
            if not narrow_path:
                narrow_path = data.get("resPath", "")
            # 绘图服务参数获取
            plot_data = data.get("plots", None)
            return light_detect_result, narrow_path, plot_data
        else:
            logger.error("访问算法接口没有成功！")
            return None, None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问算法接口错误")
        return None, None, None


# 指针仪表读数服务接口
def post_agx_pointer(photo_name, u_params_list, type=1):
    if os.path.exists(photo_name):
        with open(photo_name, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')

            input_get = {
                'picPathStr': photo_name,  # 图片路径 *
                'picData': img,  # 原始图片数据 *
                # 'picType': type,  # 图片类型（四倍1，工业2，web3）
                'picType': 1,
                'isPreview': True,
                'rois': u_params_list,
            }
    try:
        ret = requests.post(
            "http://{}:{}/algoritmic_server/meter".format(pointer_ips.get("host"), pointer_ips.get("port")),
            data=json.dumps(input_get))
        if ret.status_code == 200:
            try:
                data = json.loads(ret.text, encoding="utf-8")
                logger.info("指针仪表算法返回结果")
                logger.info(data)
            except Exception as e:
                logger.error(e)
                return None, None, None
            light_detect_result = data.get("results")  # 解析结果
            # light_path = data.get("resPath")  # 图片存储路径
            narrow_path = data.get("narrowPath", None)  # 缩略路径
            if not narrow_path:
                narrow_path = data.get("resPath", "")
            # 绘图服务参数获取
            plot_data = data.get("plots", None)
            return light_detect_result, narrow_path, plot_data
        else:
            logger.error("访问算法接口没有成功！")
            return None, None, None
    except Exception as e:
        logger.error(e)
        logger.info("访问算法接口错误")
        return None, None, None


def post_agx_plot(photo_name, u_params_list, type=1):
    if os.path.exists(photo_name):
        with open(photo_name, 'rb') as f:
            img = base64.b64encode(f.read()).decode('utf-8')
            input_get = {
                'picPathStr': photo_name,  # 图片路径 *
                'picData': img,  # 原始图片数据 *
                # 'picType': type,  # 图片类型（四倍1，工业2，web3）
                'picType': type,
                'isSaved': True,
                'plotMsg': u_params_list,
                'isReturnImg': False
            }
    try:
        ret = requests.post(
            "http://{}:{}/algoritmic_server/plot".format(pointer_ips.get("host"), pointer_ips.get("port")),
            data=json.dumps(input_get))
        if ret.status_code == 200:
            try:
                data = json.loads(ret.text, encoding="utf-8")
                logger.info("绘图算法返回结果")
                logger.info(data)
            except Exception as e:
                logger.error(e)
                return None
            light_detect_result = data.get("resPath")  # 解析结果
            return light_detect_result
        else:
            logger.error("访问算法接口没有成功！")
            return None
    except Exception as e:
        logger.error(e)
        logger.info("访问算法接口错误")
        return None
