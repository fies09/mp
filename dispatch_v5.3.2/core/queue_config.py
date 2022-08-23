import queue

rfid_queue = queue.Queue(50)

err_qrcocde_quene = queue.Queue(500) # rfid异常数据队列
err_rfid_quene = queue.Queue(500) # rfid异常数据队列

check_qrcode_quene = queue.Queue(500) # 二维码待盘点数据队列
check_rfid_quene = queue.Queue(500) # rfid待盘点设备
