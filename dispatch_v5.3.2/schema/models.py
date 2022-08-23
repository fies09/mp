# coding: utf-8
from sqlalchemy import BigInteger, Column, DateTime, Float, ForeignKey, ForeignKeyConstraint, Index, Integer, JSON, LargeBinary, Numeric, SmallInteger, String, Table, Text, text
from sqlalchemy.orm import relationship
from sqlalchemy.dialects.mysql.types import LONGBLOB, MEDIUMBLOB
from sqlalchemy.ext.declarative import declarative_base


Base = declarative_base()
metadata = Base.metadata


class AlarmManage(Base):
    __tablename__ = 'alarm_manage'

    alarm_manage_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False, server_default=text("'0'"))
    core_room_id = Column(Integer, nullable=False, server_default=text("'0'"))
    inspect_project_detail_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_device_id = Column(Integer, nullable=False)
    robot_position_id = Column(Integer, nullable=False, server_default=text("'0'"))
    core_cabinet_id = Column(Integer)
    core_device_id = Column(String(32), server_default=text("'0'"))
    core_device_part_id = Column(String(64))
    alarm_u = Column(String(20), server_default=text("'0'"))
    robot_item_id = Column(Integer, server_default=text("'0'"))
    power_cabinet_id = Column(Integer)
    power_device_id = Column(String(32))
    power_robot_item_id = Column(Integer)
    alarm_type = Column(Integer, nullable=False)
    level = Column(Integer, nullable=False)
    num = Column(Integer, nullable=False, server_default=text("'1'"))
    value = Column(String(32), nullable=False)
    alarm_desc = Column(String(500))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    user_id = Column(Integer, server_default=text("'0'"))
    alter_user = Column(Integer)
    assign = Column(Integer, nullable=False, server_default=text("'0'"))
    assign_time = Column(DateTime)
    deal_time = Column(DateTime)
    info = Column(String(500))
    img_path = Column(String(255))
    identify = Column(String(64))
    undo_status = Column(Integer, server_default=text("'0'"))
    common_num = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer)
    inspect_time = Column(DateTime)
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    inspect_project_task_id = Column(Integer)


class AlarmRule(Base):
    __tablename__ = 'alarm_rule'

    alarm_rule_id = Column(Integer, primary_key=True)
    name = Column(String(100))
    alarm_type = Column(Integer, nullable=False)
    is_default = Column(Integer, nullable=False, server_default=text("'0'"))
    is_combine = Column(Integer, nullable=False, server_default=text("'1'"))
    notice_rule_id = Column(Integer, nullable=False, server_default=text("'0'"))
    alarm_desc = Column(String(500), server_default=text("'0'"))
    undo_alarm = Column(Integer, nullable=False, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class AlarmRule1(Base):
    __tablename__ = 'alarm_rule1'

    rule_id = Column(Integer, primary_key=True)
    notice_rule_id = Column(Integer)
    name = Column(String(255))
    rule_num = Column(String(50), nullable=False)
    device_model = Column(String(64))
    device_num = Column(String(50))
    cabinet_model = Column(String(64))
    type = Column(Integer, server_default=text("'1'"))
    priority = Column(Integer, server_default=text("'0'"))
    alarm_desc = Column(String(255))
    undo_description = Column(String(255))
    acquisition_interval = Column(String(20))
    alarm_influence = Column(Integer, server_default=text("'0'"))
    trigger_alarm = Column(Integer)
    undo_alarm = Column(Integer)
    instructions = Column(String(255))
    disposal_experience = Column(String(255))
    status = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class AlarmRuleIndcamera(Base):
    __tablename__ = 'alarm_rule_indcamera'

    alarm_rule_indcamera_id = Column(Integer, primary_key=True)
    notice_rule_id = Column(Integer)
    core_device_id = Column(String(32), nullable=False)
    name = Column(String(255))
    rule_num = Column(String(50), nullable=False)
    device_model = Column(String(64))
    priority = Column(Integer, server_default=text("'0'"))
    alarm_desc = Column(String(255))
    undo_description = Column(String(255))
    acquisition_interval = Column(String(20))
    alarm_influence = Column(Integer, server_default=text("'0'"))
    trigger_alarm = Column(Integer)
    undo_alarm = Column(Integer)
    instructions = Column(String(255))
    disposal_experience = Column(String(255))
    status = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class AlarmRuleIndcameraLight(Base):
    __tablename__ = 'alarm_rule_indcamera_light'

    alarm_rule_indcamera_light_id = Column(Integer, primary_key=True)
    alarm_rule_indcamera_id = Column(ForeignKey('alarm_rule_indcamera.alarm_rule_indcamera_id'), nullable=False, index=True)
    item_name = Column(String(20), nullable=False)
    value = Column(String(20), nullable=False)
    conditions = Column(String(20), nullable=False)
    level = Column(Integer, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    alarm_rule_indcamera = relationship('AlarmRuleIndcamera')


class AlarmRuleInfo(Base):
    __tablename__ = 'alarm_rule_info'

    alarm_rule_info_id = Column(Integer, primary_key=True)
    alarm_rule_id = Column(ForeignKey('alarm_rule.alarm_rule_id'), index=True)
    item_id = Column(Integer, nullable=False)
    item_type = Column(Integer, nullable=False)
    item_name = Column(String(20), nullable=False)
    logical_condition = Column(String(20), nullable=False, server_default=text("'||'"))
    value = Column(String(20), nullable=False)
    unit = Column(String(20))
    conditions = Column(String(20), nullable=False)
    level = Column(Integer, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    alarm_rule = relationship('AlarmRule')


class AlarmRuleLight(Base):
    __tablename__ = 'alarm_rule_light'

    light_rule_id = Column(Integer, primary_key=True)
    rule_id = Column(ForeignKey('alarm_rule1.rule_id'), index=True)
    name = Column(String(50))
    item_name = Column(String(20))
    value = Column(String(20))
    conditions = Column(String(20))
    level = Column(Integer)
    status = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    rule = relationship('AlarmRule1')


class AlarmRulePosition(Base):
    __tablename__ = 'alarm_rule_position'

    alarm_rule_position_id = Column(Integer, primary_key=True)
    grade = Column(Integer, nullable=False)
    alarm_rule_id = Column(ForeignKey('alarm_rule.alarm_rule_id'), index=True)
    robot_position_id = Column(Integer)
    type = Column(Integer)
    device_id = Column(String(32))
    camera_type = Column(Integer)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    alarm_rule = relationship('AlarmRule')


class AlarmRuleRing(Base):
    __tablename__ = 'alarm_rule_ring'

    ring_rule_id = Column(Integer, primary_key=True)
    rule_id = Column(ForeignKey('alarm_rule1.rule_id'), index=True)
    item_name = Column(String(20))
    value = Column(String(20))
    conditions = Column(String(20))
    level = Column(Integer)
    status = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    rule = relationship('AlarmRule1')


class AlarmRuleShutDown(Base):
    __tablename__ = 'alarm_rule_shut_down'

    shut_down_rule_id = Column(Integer, primary_key=True)
    notice_rule_id = Column(Integer)
    name = Column(String(64))
    device_model = Column(String(255))
    level = Column(Integer, server_default=text("'0'"))
    priority = Column(Integer, server_default=text("'0'"))
    status = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class AlarmRuleShutDownCount(Base):
    __tablename__ = 'alarm_rule_shut_down_count'

    shut_down_count_rule_id = Column(Integer, primary_key=True)
    shut_down_rule_id = Column(ForeignKey('alarm_rule_shut_down.shut_down_rule_id'), nullable=False, index=True)
    indicators = Column(String(20))
    conditions = Column(String(20))
    countvalue = Column(String(20))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    shut_down_rule = relationship('AlarmRuleShutDown')


class AlarmRuleShutDownRelated(Base):
    __tablename__ = 'alarm_rule_shut_down_related'

    shut_down_related_rule_id = Column(Integer, primary_key=True)
    shut_down_rule_id = Column(ForeignKey('alarm_rule_shut_down.shut_down_rule_id'), nullable=False, index=True)
    name = Column(String(20))
    start_u = Column(String(20))
    end_u = Column(String(20))
    r_num = Column(String(20))
    y_num = Column(String(20))
    b_num = Column(String(20))
    g_num = Column(String(20))
    rule_type = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    shut_down_rule = relationship('AlarmRuleShutDown')


class ConfigAlgorithmModel(Base):
    __tablename__ = 'config_algorithm_model'

    config_algorithm_model_id = Column(Integer, primary_key=True)
    robot_algorithm_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_device_id = Column(Integer, nullable=False, server_default=text("'0'"))
    name = Column(String(20), nullable=False)
    factory = Column(String(65), nullable=False)
    type = Column(Integer, nullable=False)
    parameters = Column(JSON)
    img_path = Column(MEDIUMBLOB)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class ConfigEmail(Base):
    __tablename__ = 'config_email'

    config_email_id = Column(Integer, primary_key=True)
    title = Column(String(64), nullable=False)
    protocol = Column(String(100), nullable=False)
    host = Column(String(100), nullable=False)
    port = Column(String(100), nullable=False)
    author = Column(String(255), nullable=False)
    password = Column(String(255), nullable=False)
    emailswitch = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class ConfigMessageLog(Base):
    __tablename__ = 'config_message_log'

    config_message_log_id = Column(Integer, primary_key=True)
    message_title = Column(String(64))
    message_type = Column(String(10))
    message_content = Column(String(10000))
    content_type = Column(Integer)
    receive_user = Column(String(255))
    post_user = Column(String(255))
    send_time = Column(DateTime)


class ConfigNoticeDateRule(Base):
    __tablename__ = 'config_notice_date_rule'

    notice_date_rule_id = Column(Integer, primary_key=True)
    notice_rule_id = Column(ForeignKey('config_notice_rule.notice_rule_id'), index=True)
    start_date = Column(String(20))
    type = Column(Integer)
    start_time = Column(String(20))
    end_date = Column(String(20))
    end_time = Column(String(20))
    remark = Column(String(500))

    notice_rule = relationship('ConfigNoticeRule')


class ConfigNoticeRule(Base):
    __tablename__ = 'config_notice_rule'

    notice_rule_id = Column(Integer, primary_key=True)
    name = Column(String(64))
    level = Column(String(100))
    alarm_type = Column(String(100))
    notice_way = Column(String(100))
    user_type = Column(Integer, server_default=text("'1'"))
    notice_user = Column(String(100))
    priority = Column(Integer)
    status = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    type = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class ConfigSm(Base):
    __tablename__ = 'config_sms'

    config_sms_id = Column(Integer, primary_key=True)
    title = Column(String(64))
    account_name = Column(String(64), nullable=False)
    account_pwd = Column(String(64), nullable=False)
    url = Column(String(255), nullable=False)
    status = Column(Integer, nullable=False)
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class CoreCabinet(Base):
    __tablename__ = 'core_cabinet'

    core_cabinet_id = Column(Integer, primary_key=True)
    core_room_id = Column(ForeignKey('core_room.core_room_id'), nullable=False, index=True)
    cabinet_number = Column(String(64), nullable=False)
    name = Column(String(64), nullable=False)
    cabinet_model = Column(String(64))
    cabinet_height = Column(Float(asdecimal=True))
    cabinet_width = Column(Float(asdecimal=True))
    cabinet_u = Column(Integer, nullable=False)
    electricity = Column(String(50))
    voltage = Column(String(50))
    maxvoltage = Column(String(50))
    distance = Column(Integer, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    core_room = relationship('CoreRoom')


class CoreDevice(Base):
    __tablename__ = 'core_device'

    core_device_id = Column(String(32), primary_key=True)
    device_classify_id = Column(Integer, nullable=False, index=True)
    name = Column(String(64), nullable=False)
    device_ip = Column(String(50))
    device_number = Column(String(100), nullable=False)
    device_model = Column(String(64))
    device_sn = Column(String(64))
    device_pn = Column(String(64))
    device_sign = Column(String(255))
    device_icon = Column(String(64))
    purchase_date = Column(String(64))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class CoreDeviceLocation(Base):
    __tablename__ = 'core_device_location'

    core_device_location_id = Column(Integer, primary_key=True)
    core_device_id = Column(String(32), nullable=False)
    core_room_id = Column(Integer, index=True)
    core_cabinet_id = Column(Integer, index=True)
    start_u = Column(Integer)
    end_u = Column(Integer)
    interval_u = Column(Integer)
    occupy_u = Column(Integer)
    operator = Column(String(64))
    oper_type = Column(Integer, nullable=False)
    status = Column(Integer)
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class CoreDevicePart(Base):
    __tablename__ = 'core_device_part'

    core_device_part_id = Column(String(64), primary_key=True)
    core_device_id = Column(ForeignKey('core_device.core_device_id'), nullable=False, index=True)
    robot_position_id = Column(Integer, nullable=False, server_default=text("'0'"))
    name = Column(String(20), nullable=False)
    start_u = Column(Integer)
    end_u = Column(Integer)
    order_num = Column(Integer)
    light_color = Column(String(20))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    core_device = relationship('CoreDevice')


class CoreIndcameraParam(Base):
    __tablename__ = 'core_indcamera_param'

    core_indcamera_param_id = Column(Integer, primary_key=True)
    core_device_id = Column(String(32), nullable=False, server_default=text("'0'"))
    robot_device_id = Column(Integer, nullable=False, server_default=text("'0'"))
    config_algorithm_model_id = Column(Integer, index=True)
    alarm_rule_indcamera_id = Column(Integer, nullable=False)
    robot_algorithm_id = Column(Integer, nullable=False)
    elevator_height = Column(Integer, nullable=False)
    img_path = Column(String(500))
    img_pah_thumbnail = Column(String(500))
    device_param = Column(JSON, nullable=False)
    algorithm_param = Column(JSON)
    elevator_speed = Column(Integer, nullable=False, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    is_inventory = Column(Integer, nullable=False, server_default=text("'1'"))


class CoreInventoryDatum(Base):
    __tablename__ = 'core_inventory_data'

    inventory_data_id = Column(Integer, primary_key=True)
    inventory_record_id = Column(ForeignKey('core_inventory_record.inventory_record_id'), index=True, server_default=text("'0'"))
    robot_id = Column(Integer, nullable=False)
    robot_path_id = Column(Integer, nullable=False)
    inventory_id = Column(Integer)
    inspect_project_detail_id = Column(Integer, nullable=False)
    robot_position_id = Column(Integer)
    inventory_status = Column(Integer)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    inspect_project_task_id = Column(Integer)

    inventory_record = relationship('CoreInventoryRecord')


t_core_inventory_param = Table(
    'core_inventory_param', metadata,
    Column('core_indcamera_param_id', ForeignKey('core_indcamera_param.core_indcamera_param_id'), index=True),
    Column('robot_algorithm_id', Integer, nullable=False),
    Column('inventory_height', Integer, nullable=False, server_default=text("'0'")),
    Column('iventory_device_param', JSON, nullable=False)
)


class CoreInventoryRecord(Base):
    __tablename__ = 'core_inventory_record'

    inventory_record_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    inspect_project_detail_id = Column(Integer, nullable=False)
    cabinet_count = Column(Integer, nullable=False, server_default=text("'0'"))
    device_count = Column(Integer, nullable=False, server_default=text("'0'"))
    device_fail_count = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    inspect_project_task_id = Column(Integer)


class CoreInventoryType(Base):
    __tablename__ = 'core_inventory_type'

    inventory_id = Column(Integer, primary_key=True)
    core_device_id = Column(ForeignKey('core_device.core_device_id'), nullable=False, index=True)
    core_room_id = Column(Integer)
    core_cabinet_id = Column(Integer)
    inventory_type = Column(Integer, nullable=False, server_default=text("'0'"))
    rfid = Column(String(32))
    qrcode_path = Column(String(255))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    core_device = relationship('CoreDevice')


class CoreRfid(Base):
    __tablename__ = 'core_rfid'

    rfid_id = Column(Integer, primary_key=True)
    core_device_id = Column(ForeignKey('core_device.core_device_id'), index=True)
    core_cabinet_id = Column(Integer)
    rfid = Column(String(64), nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    core_device = relationship('CoreDevice')


class CoreRoom(Base):
    __tablename__ = 'core_room'

    core_room_id = Column(Integer, primary_key=True)
    dept_id = Column(ForeignKey('sys_dept.dept_id'), index=True)
    name = Column(String(64), nullable=False)
    room_number = Column(String(64), nullable=False)
    capacity = Column(Integer, nullable=False)
    shape = Column(JSON)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    img_path = Column(String(64))
    type = Column(Integer)

    dept = relationship('SysDept')


class InsepctDeviceIndicatorDatum(Base):
    __tablename__ = 'insepct_device_indicator_data'

    device_indicator_data_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    inspect_project_detail_id = Column(Integer, nullable=False)
    core_cabinet_id = Column(Integer, nullable=False, server_default=text("'0'"))
    core_room_id = Column(Integer, nullable=False, server_default=text("'0'"))
    core_device_id = Column(String(32), nullable=False, server_default=text("'0'"))
    core_device_part_id = Column(String(64), nullable=False, server_default=text("'0'"))
    robot_position_id = Column(Integer, nullable=False, server_default=text("'0'"))
    yellow_num = Column(Integer, nullable=False, server_default=text("'0'"))
    blue_num = Column(Integer, nullable=False, server_default=text("'0'"))
    green_num = Column(Integer, nullable=False, server_default=text("'0'"))
    red_num = Column(Integer, nullable=False, server_default=text("'0'"))
    location_u = Column(String(20), nullable=False)
    identify = Column(String(255), nullable=False)
    img_path = Column(String(255), nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class InspectDatum(Base):
    __tablename__ = 'inspect_data'

    inspect_data_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    cabinet_id = Column(Integer, nullable=False)
    cabinet_name = Column(String(64), nullable=False, server_default=text("''"))
    inspect_project_detail_id = Column(Integer, server_default=text("'-1'"))
    type = Column(Integer, nullable=False, server_default=text("'0'"))
    cabinet_type = Column(Integer, nullable=False, server_default=text("'-1'"))
    device_desc = Column(String(500), server_default=text("''"))
    dynamic_desc = Column(String(500), server_default=text("''"))
    status = Column(Integer, server_default=text("'2'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)


class InspectDataInfo(Base):
    __tablename__ = 'inspect_data_info'

    inspect_data_info_id = Column(Integer, primary_key=True)
    inspect_data_id = Column(Integer, index=True)
    cabinet_id = Column(Integer, nullable=False)
    cabinet_type = Column(Integer, server_default=text("'-1'"))
    inspect_project_detail_id = Column(Integer, nullable=False)
    robot_position_id = Column(Integer, nullable=False)
    item_id = Column(Integer, nullable=False)
    name = Column(String(20), nullable=False)
    unit = Column(String(20))
    value = Column(String(20), nullable=False)
    status = Column(Integer, nullable=False)
    alarm_rule_desc = Column(String(200))
    device_id = Column(String(32))
    device_name = Column(String(64))
    server_u = Column(String(20))
    device_classify_id = Column(Integer)
    classify_name = Column(String(64))
    part_id = Column(String(64))
    part_name = Column(String(20))
    img_path = Column(String(255))
    type = Column(Integer, nullable=False)
    start_time = Column(DateTime)
    end_tIme = Column(DateTime)


class InspectPlan(Base):
    __tablename__ = 'inspect_plan'

    inspect_plan_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    name = Column(String(20), nullable=False)
    plan_type = Column(Integer, nullable=False)
    order_num = Column(Integer, nullable=False, server_default=text("'0'"))
    start_time = Column(String(20), nullable=False)
    end_time = Column(String(20), nullable=False)
    interval_time = Column(Integer)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class InspectPlanWork(Base):
    __tablename__ = 'inspect_plan_work'

    inspect_plan_work_id = Column(Integer, primary_key=True)
    inspect_plan_id = Column(ForeignKey('inspect_plan.inspect_plan_id'), nullable=False, index=True)
    inspect_work_id = Column(Integer, index=True)
    start_time = Column(String(20))
    plan_date = Column(String(20))
    related_id = Column(Integer)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    inspect_plan = relationship('InspectPlan')


class InspectPositionItemDatum(Base):
    __tablename__ = 'inspect_position_item_data'

    position_item_data_id = Column(Integer, primary_key=True)
    inspect_project_detail_id = Column(Integer, nullable=False, server_default=text("'0'"))
    user_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_id = Column(Integer, nullable=False)
    robot_item_id = Column(Integer, nullable=False)
    robot_position_id = Column(Integer, nullable=False)
    core_cabinet_id = Column(Integer, nullable=False, server_default=text("'0'"))
    core_device_id = Column(String(32), nullable=False, server_default=text("'0'"))
    server_u = Column(String(32), nullable=False, server_default=text("'0'"))
    value = Column(String(20), nullable=False)
    img_path = Column(String(255))
    video_path = Column(String(255))
    sound_path = Column(String(255))
    type = Column(Integer, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    crop_path = Column(String(500))
    power_cabinet_id = Column(Integer, nullable=False, server_default=text("'0'"))
    play_desc = Column(Text)
    play_status = Column(Integer)
    inspect_project_task_id = Column(Integer)
    alarm_rule_id = Column(Integer)
    device_part_id = Column(String(64))
    robot_device_id = Column(Integer)


class InspectProject(Base):
    __tablename__ = 'inspect_project'

    inspect_project_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    inspect_project_task_id = Column(ForeignKey('inspect_project_task.inspect_project_task_id'), nullable=False, index=True)
    project_num = Column(String(20), nullable=False)
    cron_expression = Column(String(500), nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    intervene = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    inspect_project_task = relationship('InspectProjectTask')


class InspectProjectDetail(Base):
    __tablename__ = 'inspect_project_detail'

    inspect_project_detail_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    robot_task_type_id = Column(Integer, nullable=False)
    robot_path_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_position_id = Column(Integer, server_default=text("'0'"))
    inspect_project_id = Column(ForeignKey('inspect_project.inspect_project_id'), ForeignKey('inspect_project.inspect_project_id'), index=True)
    inspect_project_task_id = Column(Integer, nullable=False, server_default=text("'0'"))
    obstacle = Column(Integer, server_default=text("'0'"))
    num = Column(Integer, server_default=text("'0'"))
    wait_time = Column(Integer, server_default=text("'0'"))
    power_consumption = Column(Integer, nullable=False, server_default=text("'0'"))
    task_type_name = Column(String(20), nullable=False)
    task_num = Column(String(20), nullable=False)
    task_status = Column(Integer, nullable=False, server_default=text("'0'"))
    task_message = Column(String(500))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    exception_info = Column(String(500))
    is_server = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)

    inspect_project = relationship('InspectProject', primaryjoin='InspectProjectDetail.inspect_project_id == InspectProject.inspect_project_id')
    inspect_project1 = relationship('InspectProject', primaryjoin='InspectProjectDetail.inspect_project_id == InspectProject.inspect_project_id')


class InspectProjectTask(Base):
    __tablename__ = 'inspect_project_task'

    inspect_project_task_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    name = Column(String(64), nullable=False)
    task_num = Column(String(20), nullable=False)
    robot_task_type_id = Column(Integer, nullable=False)
    robot_path_id = Column(Integer, server_default=text("'0'"))
    wait_time = Column(Integer, nullable=False)
    meet_time = Column(Integer, nullable=False, server_default=text("'0'"))
    unit = Column(String(20), nullable=False)
    minimum_battery = Column(Integer, nullable=False)
    is_break = Column(Integer, nullable=False, server_default=text("'0'"))
    is_auto = Column(Integer, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class InspectProjectTaskPathRelation(Base):
    __tablename__ = 'inspect_project_task_path_relation'

    task_path_relation_id = Column(Integer, primary_key=True)
    inspect_project_task_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_path_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_position_id = Column(Integer, nullable=False, server_default=text("'0'"))
    play_desc = Column(Text)
    play_status = Column(Integer, nullable=False, server_default=text("'0'"))
    create_by = Column(String(64), server_default=text("''"))
    create_time = Column(DateTime, server_default=text("CURRENT_TIMESTAMP"))
    update_by = Column(String(64), server_default=text("''"))
    update_time = Column(DateTime, server_default=text("CURRENT_TIMESTAMP"))
    remark = Column(String(500), server_default=text("''"))


class InspectRealtimeItemDatum(Base):
    __tablename__ = 'inspect_realtime_item_data'

    realtime_data_id = Column(Integer, primary_key=True)
    user_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_cmd_id = Column(Integer, server_default=text("'0'"))
    robot_id = Column(Integer, nullable=False)
    robot_item_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_position_id = Column(Integer, nullable=False, server_default=text("'0'"))
    inspect_project_detail_id = Column(Integer, nullable=False, server_default=text("'0'"))
    position = Column(JSON, nullable=False)
    value = Column(String(20), nullable=False)
    img_path = Column(String(255), nullable=False)
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class InspectReport(Base):
    __tablename__ = 'inspect_report'

    report_id = Column(Integer, primary_key=True)
    name = Column(String(65, 'utf8_bin'))
    report_num = Column(String(64, 'utf8_bin'))
    dept_id = Column(Integer, nullable=False)
    robot_id = Column(Integer, nullable=False)
    inspect_project_detail_id = Column(Integer)
    inspect_start_time = Column(DateTime)
    inspect_end_time = Column(DateTime)
    type = Column(String(10))
    file_path = Column(String(255))
    is_power = Column(Integer, server_default=text("'1'"))
    status = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    report_desc = Column(String(255))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500, 'utf8_bin'))


class InspectRfidDeviceDatum(Base):
    __tablename__ = 'inspect_rfid_device_data'

    rfid_device_data_id = Column(Integer, primary_key=True)
    robot_path_id = Column(Integer)
    robot_cmd_id = Column(Integer)
    robot_id = Column(Integer)
    robot_item_id = Column(Integer)
    inspect_project_detail_id = Column(Integer)
    robot_position_id = Column(Integer)
    rfid = Column(String(64), nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class InspectTask(Base):
    __tablename__ = 'inspect_task'

    inspect_task_id = Column(Integer, primary_key=True)
    inspect_project_detail_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_id = Column(Integer)
    name = Column(String(20))
    robot_task_type_id = Column(Integer)
    task_name = Column(String(20))
    task_num = Column(Integer)
    results = Column(Text)
    status = Column(Integer)
    del_flag = Column(Integer, server_default=text("'0'"))
    release_time = Column(DateTime)
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class InspectTaskResult(Base):
    __tablename__ = 'inspect_task_result'

    task_result_id = Column(Integer, primary_key=True)
    inspect_task_id = Column(ForeignKey('inspect_task.inspect_task_id'), index=True)
    inspect_project_detail_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_position_id = Column(Integer)
    name = Column(String(20))
    core_cabinet_id = Column(Integer)
    cabinet_name = Column(String(64))
    move_ring_status = Column(Integer, server_default=text("'2'"))
    light_ring_status = Column(Integer, server_default=text("'2'"))
    surface_temp_status = Column(Integer, server_default=text("'2'"))
    power_cabinet_id = Column(Integer)
    dashboard_status = Column(Integer, server_default=text("'2'"))
    switch_status = Column(Integer, server_default=text("'2'"))
    pointer_dashboard_status = Column(Integer, server_default=text("'2'"))

    inspect_task = relationship('InspectTask')


class InspectTaskResultInfo(Base):
    __tablename__ = 'inspect_task_result_info'

    task_result_info_id = Column(Integer, primary_key=True)
    inspect_task_id = Column(Integer, nullable=False)
    task_result_id = Column(ForeignKey('inspect_task_result.task_result_id'), index=True)
    inspect_project_detail_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_item_id = Column(Integer)
    name = Column(String(20))
    unit = Column(String(20))
    value = Column(String(20))
    status = Column(Integer, server_default=text("'0'"))
    description = Column(String(200))
    type = Column(Integer)
    core_device_id = Column(String(32))
    device_name = Column(String(64))
    server_u = Column(String(20), server_default=text("'0'"))
    core_device_part_id = Column(String(64))
    part_name = Column(String(20))
    img_path = Column(String(255))
    video_path = Column(String(255))
    sound_path = Column(String(255))
    deal_status = Column(Integer, server_default=text("'0'"))
    inspect_time = Column(DateTime)
    power_robot_item_id = Column(Integer)
    power_device_id = Column(String(32))

    task_result = relationship('InspectTaskResult')


class InspectWork(Base):
    __tablename__ = 'inspect_work'

    inspect_work_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, index=True)
    name = Column(String(20), nullable=False)
    type = Column(Integer, nullable=False, server_default=text("'0'"))
    order_num = Column(Integer, nullable=False, server_default=text("'0'"))
    mini_charge = Column(Integer, nullable=False, server_default=text("'20'"))
    is_break = Column(Integer, nullable=False, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class InspectWorkDatum(Base):
    __tablename__ = 'inspect_work_data'

    inspect_work_data_id = Column(Integer, primary_key=True)
    robot_position_id = Column(ForeignKey('robot_position.robot_position_id'), nullable=False, index=True)
    inspect_work_id = Column(ForeignKey('inspect_work.inspect_work_id'), index=True)
    is_ring_state = Column(Integer, nullable=False, server_default=text("'1'"))
    is_temp_state = Column(Integer, nullable=False, server_default=text("'1'"))
    is_light_state = Column(Integer, nullable=False, server_default=text("'1'"))
    is_open_state = Column(Integer, nullable=False, server_default=text("'1'"))
    is_close_state = Column(Integer, nullable=False, server_default=text("'1'"))
    is_other_state = Column(Integer, server_default=text("'1'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    inspect_work = relationship('InspectWork')
    robot_position = relationship('RobotPosition')


class LogoTable(Base):
    __tablename__ = 'logo_table'

    id = Column(Integer, primary_key=True)
    logo_key = Column(String(50), nullable=False)
    logo = Column(String, nullable=False)
    create_time = Column(DateTime)


class PowerAlarmRule(Base):
    __tablename__ = 'power_alarm_rule'

    power_alarm_rule_id = Column(Integer, primary_key=True)
    notice_rule_id = Column(Integer, nullable=False)
    power_device_id = Column(String(32), nullable=False)
    name = Column(String(32), nullable=False)
    rule_num = Column(String(50), nullable=False)
    priority = Column(Integer, server_default=text("'0'"))
    alarm_desc = Column(String(255))
    undo_description = Column(String(255))
    acquisition_interval = Column(String(20))
    alarm_influence = Column(Integer, server_default=text("'0'"))
    trigger_alarm = Column(Integer)
    undo_alarm = Column(Integer)
    instructions = Column(String(255))
    disposal_experience = Column(String(255))
    status = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class PowerAlarmRuleDetail(Base):
    __tablename__ = 'power_alarm_rule_detail'

    power_alarm_rule_detail_id = Column(Integer, primary_key=True)
    power_alarm_rule_id = Column(Integer, nullable=False)
    item_name = Column(String(20), nullable=False)
    value = Column(String(20), nullable=False)
    conditions = Column(String(20), nullable=False)
    level = Column(Integer, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class PowerCabinet(Base):
    __tablename__ = 'power_cabinet'

    power_cabinet_id = Column(Integer, primary_key=True)
    core_room_id = Column(Integer, nullable=False)
    cabinet_number = Column(String(64), nullable=False)
    name = Column(String(64), nullable=False)
    cabinet_model = Column(String(64))
    cabinet_height = Column(Float(asdecimal=True))
    cabinet_width = Column(Float(asdecimal=True))
    electricity = Column(String(50))
    voltage = Column(String(50))
    maxvoltage = Column(String(50))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class PowerCameraParam(Base):
    __tablename__ = 'power_camera_param'

    power_camera_param_id = Column(Integer, primary_key=True)
    power_device_id = Column(String(32), nullable=False)
    robot_device_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_algorithm_id = Column(Integer, nullable=False, server_default=text("'0'"))
    config_algorithm_model_id = Column(Integer, nullable=False, server_default=text("'0'"))
    power_robot_item_id = Column(Integer, nullable=False, server_default=text("'0'"))
    power_alarm_rule_id = Column(Integer)
    elevator_height = Column(Integer, nullable=False, server_default=text("'0'"))
    elevator_speed = Column(Integer, nullable=False, server_default=text("'0'"))
    img_path = Column(String(255))
    img_path_thumbnail = Column(String(255))
    device_param = Column(JSON)
    algorithm_param = Column(JSON)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class PowerDevice(Base):
    __tablename__ = 'power_device'

    power_device_id = Column(String(32), primary_key=True)
    device_classify_id = Column(Integer, nullable=False)
    name = Column(String(64), nullable=False)
    device_number = Column(String(100), nullable=False)
    device_model = Column(String(64))
    device_sign = Column(String(255))
    electricity = Column(String(50))
    voltage = Column(String(50))
    maxvoltage = Column(String(50))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    type = Column(Integer, server_default=text("'0'"))
    power_panel_id = Column(String(32), server_default=text("'0'"))


class PowerDeviceLocation(Base):
    __tablename__ = 'power_device_location'

    poewr_device_location_id = Column(Integer, primary_key=True)
    power_device_id = Column(String(32), nullable=False)
    core_room_id = Column(Integer, nullable=False)
    power_cabinet_id = Column(Integer, nullable=False)
    operator = Column(String(64))
    oper_type = Column(Integer, nullable=False)
    status = Column(Integer)
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class PowerInspectDatum(Base):
    __tablename__ = 'power_inspect_data'

    power_inspect_data_id = Column(Integer, primary_key=True)
    inspect_project_detail_id = Column(Integer, nullable=False)
    user_id = Column(Integer, server_default=text("'0'"))
    robot_id = Column(Integer, nullable=False)
    power_robot_item_id = Column(Integer, nullable=False)
    robot_position_id = Column(Integer, nullable=False)
    power_cabinet_id = Column(Integer, nullable=False, server_default=text("'0'"))
    power_device_id = Column(String(32), nullable=False, server_default=text("'0'"))
    value = Column(String(20), nullable=False)
    img_path = Column(String(255))
    type = Column(Integer, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    inspect_project_task_id = Column(Integer)
    robot_device_id = Column(Integer)
    alarm_rule_id = Column(Integer)


class PowerPanel(Base):
    __tablename__ = 'power_panel'

    power_panel_id = Column(String(32), primary_key=True)
    power_cabinet_id = Column(ForeignKey('power_cabinet.power_cabinet_id'), index=True)
    name = Column(String(32), nullable=False)
    description = Column(String(200), nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    create_by = Column(Integer)
    create_time = Column(DateTime)
    update_by = Column(Integer)
    update_time = Column(DateTime)
    remark = Column(String(500))

    power_cabinet = relationship('PowerCabinet')


class PowerRobotItem(Base):
    __tablename__ = 'power_robot_item'

    power_robot_item_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    robot_algorithm_id = Column(Integer, nullable=False)
    config_algorithm_model_id = Column(Integer, nullable=False)
    name = Column(String(20), nullable=False)
    type = Column(Integer, nullable=False)
    unit = Column(String(20))
    tags = Column(JSON)
    parameters = Column(JSON)
    order_num = Column(Integer, nullable=False, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class PowerSwitchGear(Base):
    __tablename__ = 'power_switch_gear'

    switch_gear_id = Column(Integer, primary_key=True)
    switch_name = Column(String(20))
    switch_type = Column(String(20), nullable=False)
    quantile_name = Column(String(20), nullable=False, server_default=text("'二分为'"))
    quantile_key = Column(String(20), nullable=False, server_default=text("'2c/2e'"))
    quantile_type = Column(Integer, nullable=False)
    gear_info = Column(JSON, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class QrtzCalendar(Base):
    __tablename__ = 'qrtz_calendars'

    sched_name = Column(String(120), primary_key=True, nullable=False)
    calendar_name = Column(String(200), primary_key=True, nullable=False)
    calendar = Column(LargeBinary, nullable=False)


class QrtzFiredTrigger(Base):
    __tablename__ = 'qrtz_fired_triggers'

    sched_name = Column(String(120), primary_key=True, nullable=False)
    entry_id = Column(String(95), primary_key=True, nullable=False)
    trigger_name = Column(String(200), nullable=False)
    trigger_group = Column(String(200), nullable=False)
    instance_name = Column(String(200), nullable=False)
    fired_time = Column(BigInteger, nullable=False)
    sched_time = Column(BigInteger, nullable=False)
    priority = Column(Integer, nullable=False)
    state = Column(String(16), nullable=False)
    job_name = Column(String(200))
    job_group = Column(String(200))
    is_nonconcurrent = Column(String(1))
    requests_recovery = Column(String(1))


class QrtzJobDetail(Base):
    __tablename__ = 'qrtz_job_details'

    sched_name = Column(String(120), primary_key=True, nullable=False)
    job_name = Column(String(200), primary_key=True, nullable=False)
    job_group = Column(String(200), primary_key=True, nullable=False)
    description = Column(String(250))
    job_class_name = Column(String(250), nullable=False)
    is_durable = Column(String(1), nullable=False)
    is_nonconcurrent = Column(String(1), nullable=False)
    is_update_data = Column(String(1), nullable=False)
    requests_recovery = Column(String(1), nullable=False)
    job_data = Column(LargeBinary)


class QrtzLock(Base):
    __tablename__ = 'qrtz_locks'

    sched_name = Column(String(120), primary_key=True, nullable=False)
    lock_name = Column(String(40), primary_key=True, nullable=False)


class QrtzPausedTriggerGrp(Base):
    __tablename__ = 'qrtz_paused_trigger_grps'

    sched_name = Column(String(120), primary_key=True, nullable=False)
    trigger_group = Column(String(200), primary_key=True, nullable=False)


class QrtzSchedulerState(Base):
    __tablename__ = 'qrtz_scheduler_state'

    sched_name = Column(String(120), primary_key=True, nullable=False)
    instance_name = Column(String(200), primary_key=True, nullable=False)
    last_checkin_time = Column(BigInteger, nullable=False)
    checkin_interval = Column(BigInteger, nullable=False)


class QrtzTrigger(Base):
    __tablename__ = 'qrtz_triggers'
    __table_args__ = (
        ForeignKeyConstraint(['sched_name', 'job_name', 'job_group'], ['qrtz_job_details.sched_name', 'qrtz_job_details.job_name', 'qrtz_job_details.job_group']),
        Index('sched_name', 'sched_name', 'job_name', 'job_group')
    )

    sched_name = Column(String(120), primary_key=True, nullable=False)
    trigger_name = Column(String(200), primary_key=True, nullable=False)
    trigger_group = Column(String(200), primary_key=True, nullable=False)
    job_name = Column(String(200), nullable=False)
    job_group = Column(String(200), nullable=False)
    description = Column(String(250))
    next_fire_time = Column(BigInteger)
    prev_fire_time = Column(BigInteger)
    priority = Column(Integer)
    trigger_state = Column(String(16), nullable=False)
    trigger_type = Column(String(8), nullable=False)
    start_time = Column(BigInteger, nullable=False)
    end_time = Column(BigInteger)
    calendar_name = Column(String(200))
    misfire_instr = Column(SmallInteger)
    job_data = Column(LargeBinary)

    qrtz_job_detail = relationship('QrtzJobDetail')


class QrtzCronTrigger(QrtzTrigger):
    __tablename__ = 'qrtz_cron_triggers'
    __table_args__ = (
        ForeignKeyConstraint(['sched_name', 'trigger_name', 'trigger_group'], ['qrtz_triggers.sched_name', 'qrtz_triggers.trigger_name', 'qrtz_triggers.trigger_group']),
    )

    sched_name = Column(String(120), primary_key=True, nullable=False)
    trigger_name = Column(String(200), primary_key=True, nullable=False)
    trigger_group = Column(String(200), primary_key=True, nullable=False)
    cron_expression = Column(String(200), nullable=False)
    time_zone_id = Column(String(80))


class QrtzSimpleTrigger(QrtzTrigger):
    __tablename__ = 'qrtz_simple_triggers'
    __table_args__ = (
        ForeignKeyConstraint(['sched_name', 'trigger_name', 'trigger_group'], ['qrtz_triggers.sched_name', 'qrtz_triggers.trigger_name', 'qrtz_triggers.trigger_group']),
    )

    sched_name = Column(String(120), primary_key=True, nullable=False)
    trigger_name = Column(String(200), primary_key=True, nullable=False)
    trigger_group = Column(String(200), primary_key=True, nullable=False)
    repeat_count = Column(BigInteger, nullable=False)
    repeat_interval = Column(BigInteger, nullable=False)
    times_triggered = Column(BigInteger, nullable=False)


class QrtzBlobTrigger(QrtzTrigger):
    __tablename__ = 'qrtz_blob_triggers'
    __table_args__ = (
        ForeignKeyConstraint(['sched_name', 'trigger_name', 'trigger_group'], ['qrtz_triggers.sched_name', 'qrtz_triggers.trigger_name', 'qrtz_triggers.trigger_group']),
    )

    sched_name = Column(String(120), primary_key=True, nullable=False)
    trigger_name = Column(String(200), primary_key=True, nullable=False)
    trigger_group = Column(String(200), primary_key=True, nullable=False)
    blob_data = Column(LargeBinary)


class QrtzSimpropTrigger(QrtzTrigger):
    __tablename__ = 'qrtz_simprop_triggers'
    __table_args__ = (
        ForeignKeyConstraint(['sched_name', 'trigger_name', 'trigger_group'], ['qrtz_triggers.sched_name', 'qrtz_triggers.trigger_name', 'qrtz_triggers.trigger_group']),
    )

    sched_name = Column(String(120), primary_key=True, nullable=False)
    trigger_name = Column(String(200), primary_key=True, nullable=False)
    trigger_group = Column(String(200), primary_key=True, nullable=False)
    str_prop_1 = Column(String(512))
    str_prop_2 = Column(String(512))
    str_prop_3 = Column(String(512))
    int_prop_1 = Column(Integer)
    int_prop_2 = Column(Integer)
    long_prop_1 = Column(BigInteger)
    long_prop_2 = Column(BigInteger)
    dec_prop_1 = Column(Numeric(13, 4))
    dec_prop_2 = Column(Numeric(13, 4))
    bool_prop_1 = Column(String(1))
    bool_prop_2 = Column(String(1))


class Robot(Base):
    __tablename__ = 'robot'

    robot_id = Column(Integer, primary_key=True)
    robot_type_id = Column(ForeignKey('robot_type.robot_type_id'), index=True)
    core_room_id = Column(ForeignKey('core_room.core_room_id'), index=True)
    dept_id = Column(ForeignKey('sys_dept.dept_id'), index=True)
    name = Column(String(20), nullable=False, index=True)
    hardware_config = Column(String(255), nullable=False, server_default=text("'激光导航版,硬件版本v2.0,bom编号v2.0 comment 硬件信息'"))
    software_info = Column(String(255), nullable=False, server_default=text("'软件版本v2.0,软件bom编号v2.0 comment 软件信息'"))
    production_info = Column(String(255), nullable=False)
    info = Column(String(50))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    core_room = relationship('CoreRoom')
    dept = relationship('SysDept')
    robot_type = relationship('RobotType')


class RobotAlgorithm(Base):
    __tablename__ = 'robot_algorithm'

    robot_algorithm_id = Column(Integer, primary_key=True)
    type = Column(Integer, nullable=False)
    name = Column(String(200), nullable=False, index=True)
    parameters = Column(JSON, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    order_num = Column(Integer, server_default=text("'0'"))


class RobotAlgorithmParam(Base):
    __tablename__ = 'robot_algorithm_param'

    robot_algorithm_param_id = Column(Integer, primary_key=True)
    robot_algorithm_id = Column(ForeignKey('robot_algorithm.robot_algorithm_id'), nullable=False, index=True)
    parameters = Column(JSON, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot_algorithm = relationship('RobotAlgorithm')


class RobotArm(Base):
    __tablename__ = 'robot_arm'

    robot_arm_id = Column(Integer, primary_key=True)
    robot_id = Column(ForeignKey('robot.robot_id'), nullable=False, index=True)
    name = Column(String(20), nullable=False, index=True)
    zero_position = Column(String(500), server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot = relationship('Robot')


class RobotArmParam(Base):
    __tablename__ = 'robot_arm_param'

    robot_arm_param_id = Column(Integer, primary_key=True)
    robot_arm_id = Column(ForeignKey('robot_arm.robot_arm_id'), nullable=False, index=True)
    goal = Column(String(500), nullable=False, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot_arm = relationship('RobotArm')


class RobotAutomaticInspect(Base):
    __tablename__ = 'robot_automatic_inspect'

    robot_automatic_inspect_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    status = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class RobotAutomaticInspectDetail(Base):
    __tablename__ = 'robot_automatic_inspect_detail'

    robot_automatic_inspect_detail_id = Column(Integer, primary_key=True)
    robot_automatic_inspect_id = Column(Integer, nullable=False)
    inspect_item_type = Column(Integer, nullable=False)
    inspect_detail = Column(JSON)
    status = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class RobotCmd(Base):
    __tablename__ = 'robot_cmd'

    robot_cmd_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    user_id = Column(Integer, nullable=False)
    robot_task_type_id = Column(Integer, nullable=False)
    robot_path_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_position_id = Column(Integer, nullable=False, server_default=text("'0'"))
    name = Column(String(20), nullable=False)
    parameters = Column(JSON, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    obstacle = Column(Integer, server_default=text("'0'"))
    is_server = Column(Integer, server_default=text("'0'"))


class RobotConfig(Base):
    __tablename__ = 'robot_config'

    robot_config_id = Column(Integer, primary_key=True)
    robot_id = Column(ForeignKey('robot.robot_id'), index=True)
    name = Column(String(20), index=True)
    mini_charge = Column(Integer, nullable=False, server_default=text("'20'"))
    start_time = Column(String(20), nullable=False, server_default=text("'00:00'"))
    end_time = Column(String(20), server_default=text("'23:59'"))
    patrol_period = Column(Integer, server_default=text("'3600'"))
    db_address = Column(String(15), nullable=False)
    ssh_user = Column(String(10), nullable=False)
    db_port = Column(String(10), nullable=False)
    db_name = Column(String(20), nullable=False)
    db_user = Column(String(10), nullable=False)
    db_password = Column(String(20), nullable=False)
    ros_adress = Column(String(20), nullable=False)
    ros_port = Column(String(6), nullable=False)
    ros_info = Column(JSON, nullable=False)
    files_path = Column(String(255), nullable=False)
    holding_days = Column(Integer, nullable=False, server_default=text("'10'"))
    parameters = Column(JSON, nullable=False)
    sync_data_switch = Column(Integer, nullable=False, server_default=text("'1'"))
    tags = Column(JSON)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot = relationship('Robot')


class RobotDevice(Base):
    __tablename__ = 'robot_device'

    robot_device_id = Column(Integer, primary_key=True)
    robot_device_type_id = Column(ForeignKey('robot_device_type.robot_device_type_id'), index=True)
    robot_id = Column(ForeignKey('robot.robot_id'), index=True)
    name = Column(String(20), nullable=False, index=True)
    address = Column(String(20), nullable=False)
    port = Column(Integer, nullable=False)
    username = Column(String(20), nullable=False)
    passwd = Column(String(32), nullable=False)
    code = Column(String(32))
    factory = Column(String(20), nullable=False)
    status = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    order_num = Column(Integer, server_default=text("'0'"))

    robot_device_type = relationship('RobotDeviceType')
    robot = relationship('Robot')


class RobotDeviceCamera(Base):
    __tablename__ = 'robot_device_camera'

    robot_device_camera_id = Column(Integer, primary_key=True)
    robot_id = Column(ForeignKey('robot.robot_id'), index=True)
    robot_device_id = Column(ForeignKey('robot_device.robot_device_id'), index=True)
    camera_name = Column(String(64), nullable=False)
    login_name = Column(String(64), nullable=False)
    camera_pwd = Column(String(64), nullable=False)
    camera_ip = Column(String(32), nullable=False)
    camera_port = Column(String(20), nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    type = Column(Integer, nullable=False)
    channel = Column(String(30))
    mobile_port = Column(String(20))
    port_type = Column(Integer, nullable=False)
    stream_type = Column(Integer)
    create_by = Column(Integer)
    create_time = Column(DateTime)
    update_by = Column(Integer)
    update_time = Column(DateTime)
    remark = Column(String(500))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))

    robot_device = relationship('RobotDevice')
    robot = relationship('Robot')


class RobotDeviceParam(Base):
    __tablename__ = 'robot_device_param'

    robot_device_param_id = Column(Integer, primary_key=True)
    robot_device_id = Column(ForeignKey('robot_device.robot_device_id'), nullable=False, index=True)
    parameters = Column(JSON, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot_device = relationship('RobotDevice')


class RobotDevicePreset(Base):
    __tablename__ = 'robot_device_preset'

    preset_id = Column(Integer, primary_key=True)
    robot_ptz_id = Column(ForeignKey('robot_ptz.robot_ptz_id'), nullable=False, index=True)
    num = Column(Integer, nullable=False)
    name = Column(String(20), nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot_ptz = relationship('RobotPtz')


class RobotDeviceType(Base):
    __tablename__ = 'robot_device_type'

    robot_device_type_id = Column(Integer, primary_key=True)
    name = Column(String(20), nullable=False, index=True)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class RobotElevator(Base):
    __tablename__ = 'robot_elevator'

    robot_elevator_id = Column(Integer, primary_key=True)
    robot_id = Column(ForeignKey('robot.robot_id'), nullable=False, index=True)
    name = Column(String(20), nullable=False, index=True)
    zero_position = Column(Integer, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot = relationship('Robot')


class RobotElevatorParam(Base):
    __tablename__ = 'robot_elevator_param'

    robot_elevator_param_id = Column(Integer, primary_key=True)
    robot_elevator_id = Column(ForeignKey('robot_elevator.robot_elevator_id'), nullable=False, index=True)
    height = Column(Integer, nullable=False, server_default=text("'0'"))
    speed = Column(Integer, nullable=False, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot_elevator = relationship('RobotElevator')


class RobotItem(Base):
    __tablename__ = 'robot_item'

    robot_item_id = Column(Integer, primary_key=True)
    robot_algorithm_id = Column(ForeignKey('robot_algorithm.robot_algorithm_id'), index=True)
    robot_id = Column(ForeignKey('robot.robot_id'), index=True)
    name = Column(String(20), nullable=False)
    type = Column(Integer, nullable=False)
    unit = Column(String(20))
    tags = Column(JSON)
    parameters = Column(JSON)
    order_num = Column(Integer, nullable=False, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot_algorithm = relationship('RobotAlgorithm')
    robot = relationship('Robot')


class RobotMap(Base):
    __tablename__ = 'robot_map'

    robot_map_id = Column(Integer, primary_key=True)
    robot_id = Column(ForeignKey('robot.robot_id'), nullable=False, index=True)
    name = Column(String(32), nullable=False)
    type = Column(Integer, nullable=False)
    description = Column(String(100))
    original_path = Column(String(255))
    map_path = Column(String(255))
    target_path = Column(String(255))
    use_status = Column(Integer)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot = relationship('Robot')


class RobotPath(Base):
    __tablename__ = 'robot_path'

    robot_path_id = Column(Integer, primary_key=True)
    robot_id = Column(ForeignKey('robot.robot_id'), nullable=False, index=True)
    name = Column(String(110), nullable=False)
    path_type = Column(Integer)
    listorder = Column(Integer, nullable=False, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot = relationship('Robot')


class RobotPathPositionItem(Base):
    __tablename__ = 'robot_path_position_item'

    path_position_item_id = Column(Integer, primary_key=True)
    robot_elevator_param_id = Column(Integer, nullable=False, index=True, server_default=text("'0'"))
    robot_ptz_param_id = Column(Integer, nullable=False, index=True, server_default=text("'0'"))
    robot_device_param_id = Column(Integer, nullable=False, index=True, server_default=text("'0'"))
    robot_algorithm_param_id = Column(Integer, nullable=False, index=True, server_default=text("'0'"))
    robot_position_id = Column(ForeignKey('robot_position.robot_position_id'), index=True)
    mode = Column(Integer, server_default=text("'-1'"))
    robot_arm_param_id = Column(Integer, server_default=text("'0'"))
    robot_item_id = Column(ForeignKey('robot_item.robot_item_id'), index=True)
    name = Column(String(20), nullable=False)
    listorder = Column(Integer, nullable=False, server_default=text("'0'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot_item = relationship('RobotItem')
    robot_position = relationship('RobotPosition')


class RobotPathPositionRelation(Base):
    __tablename__ = 'robot_path_position_relation'

    path_position_relation_id = Column(Integer, primary_key=True)
    robot_path_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_position_id = Column(Integer, nullable=False, server_default=text("'0'"))
    order_num = Column(Integer, server_default=text("'0'"))
    create_by = Column(String(64), server_default=text("''"))
    create_time = Column(DateTime, server_default=text("CURRENT_TIMESTAMP"))
    update_by = Column(String(64), server_default=text("''"))
    update_time = Column(DateTime, server_default=text("CURRENT_TIMESTAMP"))
    remark = Column(String(500), server_default=text("''"))


class RobotPosition(Base):
    __tablename__ = 'robot_position'

    robot_position_id = Column(Integer, primary_key=True)
    robot_id = Column(ForeignKey('robot.robot_id'), nullable=False, index=True)
    device_id = Column(Integer, nullable=False, server_default=text("'0'"))
    name = Column(String(20), nullable=False)
    position = Column(JSON, nullable=False)
    type = Column(Integer, nullable=False)
    direction = Column(JSON)
    core_cabinet_id = Column(Integer, server_default=text("'0'"))
    current_state = Column(Integer, nullable=False, server_default=text("'0'"))
    is_obstacle = Column(Integer, nullable=False, server_default=text("'0'"))
    is_around = Column(Integer, nullable=False, server_default=text("'0'"))
    is_trunning = Column(Integer, server_default=text("'1'"))
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    distance = Column(Integer)
    power_cabinet_id = Column(Integer)
    camera_config = Column(Integer)

    robot = relationship('Robot')


class RobotPositionRelation(Base):
    __tablename__ = 'robot_position_relation'

    robot_position_id = Column(Integer, primary_key=True, nullable=False)
    related_id = Column(Integer, primary_key=True, nullable=False)


class RobotPositionRoute(Base):
    __tablename__ = 'robot_position_route'

    position_route_id = Column(Integer, primary_key=True)
    begin_id = Column(Integer, nullable=False)
    stop_id = Column(Integer, nullable=False)
    distance = Column(Float, nullable=False)
    create_time = Column(DateTime)


class RobotPtz(Base):
    __tablename__ = 'robot_ptz'

    robot_ptz_id = Column(Integer, primary_key=True)
    robot_id = Column(ForeignKey('robot.robot_id'), nullable=False, index=True)
    name = Column(String(20), nullable=False, index=True)
    address = Column(String(20), nullable=False)
    factory = Column(String(20), nullable=False)
    port = Column(Integer, nullable=False)
    username = Column(String(20), nullable=False)
    passwd = Column(String(32), nullable=False)
    code = Column(String(32))
    config_path = Column(String(255), nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot = relationship('Robot')


class RobotPtzParam(Base):
    __tablename__ = 'robot_ptz_param'

    robot_ptz_param_id = Column(Integer, primary_key=True)
    robot_ptz_id = Column(ForeignKey('robot_ptz.robot_ptz_id'), nullable=False, index=True)
    preset_num = Column(SmallInteger, nullable=False)
    tilt_position = Column(String(20), nullable=False)
    pan_position = Column(String(20), nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot_ptz = relationship('RobotPtz')


class RobotState(Base):
    __tablename__ = 'robot_state'

    robot_state_id = Column(Integer, primary_key=True)
    robot_id = Column(ForeignKey('robot.robot_id'), index=True)
    task_type_id = Column(Integer, nullable=False, server_default=text("'0'"))
    robot_temperature = Column(Integer, nullable=False, server_default=text("'0'"))
    battery_level = Column(Integer, nullable=False, server_default=text("'0'"))
    battery_temperature = Column(Integer, nullable=False, server_default=text("'0'"))
    cpu_percent = Column(Integer, nullable=False, server_default=text("'0'"))
    cpu_temperature = Column(Integer, nullable=False, server_default=text("'0'"))
    disk_percent = Column(Integer, nullable=False, server_default=text("'0'"))
    memory_percent = Column(Integer, nullable=False, server_default=text("'0'"))
    is_charged = Column(Integer, nullable=False, server_default=text("'0'"))
    obstacle_state = Column(Integer, nullable=False, server_default=text("'0'"))
    radar_state = Column(Integer, nullable=False, server_default=text("'0'"))
    motor_state = Column(Integer, nullable=False, server_default=text("'0'"))
    imu_state = Column(Integer, nullable=False, server_default=text("'0'"))
    gas_sensor_state = Column(Integer, nullable=False, server_default=text("'0'"))
    lidar_state = Column(Integer, nullable=False, server_default=text("'0'"))
    camera_state = Column(Integer, nullable=False, server_default=text("'0'"))
    thermal_state = Column(Integer, nullable=False, server_default=text("'0'"))
    battery_state = Column(Integer, nullable=False, server_default=text("'0'"))
    wind_state = Column(Integer, nullable=False, server_default=text("'0'"))
    mic_state = Column(Integer, nullable=False, server_default=text("'0'"))
    discharge_state = Column(Integer, nullable=False, server_default=text("'0'"))
    motion_info = Column(JSON, nullable=False)
    info = Column(String(50), nullable=False)
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    remark = Column(String(500))

    robot = relationship('Robot')


class RobotTask(Base):
    __tablename__ = 'robot_task'

    robot_task_id = Column(Integer, primary_key=True)
    robot_task_type_id = Column(ForeignKey('robot_task_type.robot_task_type_id'), index=True)
    robot_id = Column(Integer, nullable=False)
    robot_path_id = Column(Integer, nullable=False, server_default=text("'0'"))
    num = Column(Integer, nullable=False)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))

    robot_task_type = relationship('RobotTaskType')


class RobotTaskType(Base):
    __tablename__ = 'robot_task_type'

    robot_task_type_id = Column(Integer, primary_key=True)
    name = Column(String(20), nullable=False, index=True)
    num = Column(Integer, nullable=False, index=True)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))
    type = Column(Integer, server_default=text("'0'"))


class RobotType(Base):
    __tablename__ = 'robot_type'

    robot_type_id = Column(Integer, primary_key=True)
    name = Column(String(20), nullable=False, index=True)
    status = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class RobotVoice(Base):
    __tablename__ = 'robot_voice'

    robot_voice_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, nullable=False)
    name = Column(String(255), nullable=False)
    content = Column(String(255))
    trigger_mode = Column(Integer, server_default=text("'0'"))
    is_custom = Column(Integer, nullable=False, server_default=text("'0'"))
    is_play = Column(Integer, nullable=False, server_default=text("'0'"))
    del_flag = Column(Integer, nullable=False, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(500))


class SysConfig(Base):
    __tablename__ = 'sys_config'

    config_id = Column(Integer, primary_key=True)
    config_name = Column(String(100), server_default=text("''"))
    config_key = Column(String(100), server_default=text("''"))
    config_value = Column(String(500), server_default=text("''"))
    config_type = Column(String(1), server_default=text("'N'"))
    create_by = Column(String(64), server_default=text("''"))
    create_time = Column(DateTime)
    update_by = Column(String(64), server_default=text("''"))
    update_time = Column(DateTime)
    remark = Column(String(500))


class SysDept(Base):
    __tablename__ = 'sys_dept'

    dept_id = Column(Integer, primary_key=True)
    parent_id = Column(Integer, server_default=text("'0'"))
    ancestors = Column(String(50), server_default=text("''"))
    dept_name = Column(String(30), server_default=text("''"))
    order_num = Column(Integer, server_default=text("'0'"))
    leader = Column(String(20))
    phone = Column(String(11))
    email = Column(String(50))
    status = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_by = Column(Integer)
    create_time = Column(DateTime)
    update_by = Column(Integer)
    update_time = Column(DateTime)


class SysDeviceClassify(Base):
    __tablename__ = 'sys_device_classify'

    device_classify_id = Column(Integer, primary_key=True)
    name = Column(String(64), nullable=False)
    parent_id = Column(Integer, nullable=False, server_default=text("'0'"))
    type = Column(Integer)
    status = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    create_time = Column(DateTime)
    update_time = Column(DateTime)
    create_by = Column(Integer)
    update_by = Column(Integer)
    remark = Column(String(255))


class SysDictDatum(Base):
    __tablename__ = 'sys_dict_data'

    dict_code = Column(Integer, primary_key=True)
    dict_sort = Column(Integer, server_default=text("'0'"))
    dict_label = Column(String(100), server_default=text("''"))
    dict_value = Column(String(100), server_default=text("''"))
    dict_type = Column(String(100), server_default=text("''"))
    css_class = Column(String(100))
    list_class = Column(String(100))
    is_default = Column(String(1), server_default=text("'N'"))
    status = Column(Integer, server_default=text("'0'"))
    create_by = Column(Integer)
    create_time = Column(DateTime)
    update_by = Column(Integer)
    update_time = Column(DateTime)
    remark = Column(String(500))


class SysDictType(Base):
    __tablename__ = 'sys_dict_type'

    dict_id = Column(Integer, primary_key=True)
    dict_name = Column(String(100), server_default=text("''"))
    dict_type = Column(String(100), unique=True, server_default=text("''"))
    status = Column(Integer, server_default=text("'0'"))
    create_by = Column(Integer)
    create_time = Column(DateTime)
    update_by = Column(Integer)
    update_time = Column(DateTime)
    remark = Column(String(500))


class SysJob(Base):
    __tablename__ = 'sys_job'

    job_id = Column(Integer, primary_key=True, nullable=False)
    job_name = Column(String(64), primary_key=True, nullable=False, server_default=text("''"))
    job_group = Column(String(64), primary_key=True, nullable=False, server_default=text("'DEFAULT'"))
    invoke_target = Column(String(500), nullable=False)
    cron_expression = Column(String(255), server_default=text("''"))
    misfire_policy = Column(String(20), server_default=text("'3'"))
    concurrent = Column(String(1), server_default=text("'1'"))
    status = Column(String(1), server_default=text("'0'"))
    create_by = Column(String(64), server_default=text("''"))
    create_time = Column(DateTime)
    update_by = Column(String(64), server_default=text("''"))
    update_time = Column(DateTime)
    remark = Column(String(500), server_default=text("''"))


class SysJobLog(Base):
    __tablename__ = 'sys_job_log'

    job_log_id = Column(Integer, primary_key=True)
    job_name = Column(String(64), nullable=False)
    job_group = Column(String(64), nullable=False)
    invoke_target = Column(String(500), nullable=False)
    job_message = Column(String(500))
    status = Column(String(1), server_default=text("'0'"))
    exception_info = Column(String(2000), server_default=text("''"))
    create_time = Column(DateTime)


class SysLogininfor(Base):
    __tablename__ = 'sys_logininfor'

    info_id = Column(Integer, primary_key=True)
    login_name = Column(String(50), server_default=text("''"))
    ipaddr = Column(String(50), server_default=text("''"))
    login_location = Column(String(255), server_default=text("''"))
    browser = Column(String(50), server_default=text("''"))
    os = Column(String(50), server_default=text("''"))
    status = Column(String(1), server_default=text("'0'"))
    msg = Column(String(255), server_default=text("''"))
    login_time = Column(DateTime)


class SysMenu(Base):
    __tablename__ = 'sys_menu'

    menu_id = Column(Integer, primary_key=True)
    menu_name = Column(String(50), nullable=False)
    parent_id = Column(Integer, server_default=text("'0'"))
    order_num = Column(Integer, server_default=text("'0'"))
    url = Column(String(200), server_default=text("'#'"))
    target = Column(String(20), server_default=text("''"))
    menu_type = Column(String(1), server_default=text("''"))
    visible = Column(Integer, server_default=text("'0'"))
    is_refresh = Column(Integer, server_default=text("'1'"))
    perms = Column(String(100))
    icon = Column(String(100), server_default=text("'#'"))
    create_by = Column(Integer)
    create_time = Column(DateTime)
    update_by = Column(Integer)
    update_time = Column(DateTime)
    remark = Column(String(500), server_default=text("''"))


class SysOperLog(Base):
    __tablename__ = 'sys_oper_log'

    oper_id = Column(Integer, primary_key=True)
    title = Column(String(50), server_default=text("''"))
    business_type = Column(Integer, server_default=text("'0'"))
    method = Column(String(100), server_default=text("''"))
    request_method = Column(String(10), server_default=text("''"))
    operator_type = Column(Integer, server_default=text("'0'"))
    oper_name = Column(String(50), server_default=text("''"))
    dept_name = Column(String(50), server_default=text("''"))
    oper_url = Column(String(255), server_default=text("''"))
    oper_ip = Column(String(50), server_default=text("''"))
    oper_location = Column(String(255), server_default=text("''"))
    oper_param = Column(String(2000), server_default=text("''"))
    json_result = Column(String(2000), server_default=text("''"))
    status = Column(Integer, server_default=text("'0'"))
    error_msg = Column(String(2000), server_default=text("''"))
    oper_time = Column(DateTime)


class SysPost(Base):
    __tablename__ = 'sys_post'

    post_id = Column(Integer, primary_key=True)
    post_code = Column(String(64), nullable=False)
    post_name = Column(String(50), nullable=False)
    post_sort = Column(Integer, nullable=False)
    status = Column(Integer, server_default=text("'0'"))
    create_by = Column(Integer)
    create_time = Column(DateTime)
    update_by = Column(Integer)
    update_time = Column(DateTime)
    remark = Column(String(500))


class SysRole(Base):
    __tablename__ = 'sys_role'

    role_id = Column(Integer, primary_key=True)
    role_name = Column(String(30), nullable=False)
    role_key = Column(String(100), nullable=False)
    role_sort = Column(Integer, nullable=False)
    data_scope = Column(Integer, server_default=text("'1'"))
    status = Column(Integer, nullable=False)
    del_flag = Column(Integer, server_default=text("'0'"))
    create_by = Column(Integer)
    create_time = Column(DateTime)
    update_by = Column(Integer)
    update_time = Column(DateTime)
    remark = Column(String(500))


class SysRoleDept(Base):
    __tablename__ = 'sys_role_dept'

    role_id = Column(Integer, primary_key=True, nullable=False)
    dept_id = Column(Integer, primary_key=True, nullable=False)


class SysRoleMenu(Base):
    __tablename__ = 'sys_role_menu'

    role_id = Column(Integer, primary_key=True, nullable=False)
    menu_id = Column(Integer, primary_key=True, nullable=False)


class SysRptFile(Base):
    __tablename__ = 'sys_rpt_file'

    id = Column(String(64, 'utf8mb4_bin'), primary_key=True)
    name = Column(String(255, 'utf8mb4_bin'), nullable=False, server_default=text("''"))
    content = Column(LONGBLOB, nullable=False)
    create_time = Column(DateTime, server_default=text("CURRENT_TIMESTAMP"))
    update_time = Column(DateTime, server_default=text("CURRENT_TIMESTAMP"))


class SysUser(Base):
    __tablename__ = 'sys_user'

    user_id = Column(Integer, primary_key=True)
    dept_id = Column(Integer)
    login_name = Column(String(30), nullable=False)
    user_name = Column(String(30), server_default=text("''"))
    user_type = Column(String(2), server_default=text("'00'"))
    email = Column(String(50), server_default=text("''"))
    phonenumber = Column(String(11), server_default=text("''"))
    sex = Column(Integer, server_default=text("'0'"))
    avatar = Column(String(200), server_default=text("''"))
    password = Column(String(100), server_default=text("''"))
    salt = Column(String(20), server_default=text("''"))
    status = Column(Integer, server_default=text("'0'"))
    del_flag = Column(Integer, server_default=text("'0'"))
    login_ip = Column(String(50), server_default=text("''"))
    login_date = Column(DateTime)
    pwd_update_date = Column(DateTime)
    create_by = Column(Integer)
    create_time = Column(DateTime)
    update_by = Column(Integer)
    update_time = Column(DateTime)
    remark = Column(String(500))


class SysUserPost(Base):
    __tablename__ = 'sys_user_post'

    user_id = Column(Integer, primary_key=True, nullable=False)
    post_id = Column(Integer, primary_key=True, nullable=False)


class SysUserRole(Base):
    __tablename__ = 'sys_user_role'

    user_id = Column(Integer, primary_key=True, nullable=False)
    role_id = Column(Integer, primary_key=True, nullable=False)


t_tabchnnl = Table(
    'tabchnnl', metadata,
    Column('Type', Integer),
    Column('Time', String(127)),
    Column('User', String(127)),
    Column('UserRemote', String(127)),
    Column('AddrLogin', String(127)),
    Column('AddrLoginRemote', String(127)),
    Column('AddrChannel', String(127)),
    Column('AddrChannelRemote', String(127))
)


t_tabdevice = Table(
    'tabdevice', metadata,
    Column('DevID', String(127)),
    Column('Pass', String(127)),
    Column('Cmmt', String)
)


class Tabgroup(Base):
    __tablename__ = 'tabgroup'

    pgGroup = Column(String(127), primary_key=True)
    MemberList = Column(String)
    Cmmt = Column(String)


class Tabuser(Base):
    __tablename__ = 'tabuser'

    User = Column(String(127), primary_key=True)
    Pass = Column(String(127))
    Domain = Column(String(127))
    Email = Column(String(127))
    Cmmt = Column(String)
    Type = Column(Integer)
    Status = Column(String(127))
    Addr = Column(String(127))
    NatType = Column(Integer)
    Client = Column(String(127))
    RegTime = Column(String(127))
    LoginTime = Column(String(127))
    LoginCount = Column(Integer)
    ClientInfo = Column(String)
    Config = Column(String)
    Store = Column(String)
    HomeID = Column(String)
    GroupList = Column(String)
    ExpireBegin = Column(String(127))
    ExpireEnd = Column(String(127))
    Field20 = Column(String)
    Field21 = Column(String)
    Field22 = Column(String)
    Field23 = Column(String)
