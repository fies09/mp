# # coding: utf-8
# import logging
#
# import pytz
# from apscheduler.events import EVENT_JOB_EXECUTED, EVENT_JOB_ERROR
# from apscheduler.executors.pool import ProcessPoolExecutor, ThreadPoolExecutor
# from apscheduler.schedulers.background import BackgroundScheduler
#
#
# def my_listener(event):
#     if event.exception:
#         print("[!] The job crashed :(")
#         print(event.exception)
#     else:
#         print("[ ] The job worked")
#
#
# def base_scheduler():
#     local_time_zone = pytz.timezone('Asia/Shanghai')
#
#     executors = {
#         'default': ThreadPoolExecutor(100),
#         'processpool': ProcessPoolExecutor(64)
#     }
#
#     job_defaults = {
#         'coalesce': True,  # 错过执行的任务合并执行
#         'max_instances': 1,  # id 相同的任务实例数
#         'misfire_grace_time': 600
#     }
#
#     bscheduler = BackgroundScheduler()
#     bscheduler._logger = logging
#
#     try:
#         bscheduler.configure(executors=executors, job_defaults=job_defaults, timezone=local_time_zone)
#         bscheduler.add_listener(my_listener, EVENT_JOB_EXECUTED | EVENT_JOB_ERROR)
#         bscheduler.start()
#     except Exception as e:
#         print(e)
#
#     return bscheduler
#
#
# def print_task():
#     results = scheduler.get_jobs()
#
#     print('[+] scheduled task list')
#
#     for job in results:
#         print("[ ] {} - {}".format(str(job.next_run_time), job.args[0]['name']))
#
#
# scheduler = base_scheduler()
#
# __all__ = ['scheduler', 'print_task']
