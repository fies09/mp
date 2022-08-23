## python版本，环境版本
使用Python3.5.2版本进行开发

环境为ubuntu16.04.7

## 命名规范
class类是用驼峰的命名方式，需将继承代码写全。例如：ClassObjects(object):
def函数使用下划线连接命名，一个方法名中不可出现3个以上下划线，开头结尾不可使用下划线。例如：def function_method():

## 目录结构

- core/ 为连接路径
- modules/ 为python调用代码，其中一个元器件写一个.py文件调度
- so_pack/ 为.so文件包
- schema/ 为数据库的orm关系映射，插件为sqlalchemy。
- utils/ 为写好的使用工具，多次复用工具
- video_path 存储视频文件信息

## 返回值定义
- 在utils下的msg中
```
msg = {'code': 1,
       'status': True,
       'msg': '',
        'data': {
            'meta': {},
            'result': {},
            }
        }
```
其中 code 为状态码 1为正常 ，0为error，其他数字为特殊异常（自定义）。

return格式
```
return msg.meta().success(msg='**成功', result=results).json()
```
其中meta()中可以传入字典值，msg中成功失败的说明信息，result返回所需数据。

## 异常处理
    try:

    except Exception as e:
        logger.error(e)
    

在 logger中记录error信息。

## 注释
```
单行使用:
# 进行注释 

多行使用:
“”“这是第一行
这是第二行
这是第三行
”“”
```

## 通讯方式
使用ros action通信方式
