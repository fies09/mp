import subprocess
import time

import psutil


def get_hdd():
    p = subprocess.check_output(
        ['df', '-Tlm', '--total', '-t', 'ext4', '-t', 'ext3', '-t', 'ext2', '-t', 'reiserfs', '-t', 'jfs', '-t',
         'ntfs',
         '-t', 'fat32', '-t', 'btrfs', '-t', 'fuseblk', '-t', 'zfs', '-t', 'simfs', '-t', 'xfs']).decode("Utf-8")
    total = p.splitlines()[-1]
    used = total.split()[3]
    size = total.split()[2]
    return int(size), int(used)


def start_client():
    monitor = {}
    memory = psutil.virtual_memory()
    CPU = psutil.cpu_count()
    USE_CPU = psutil.cpu_percent()
    MEMORY_TOTAL = memory.total
    MEMORY_USED = memory.used
    MEMORY_USE = float(memory.used) / float(memory.total) * 100
    HDDTotal, HDDUsed = get_hdd()
    HDD_USE = (HDDUsed / HDDTotal) * 100
    TEMP = psutil.sensors_temperatures()
    entry = TEMP["coretemp"][0]

    monitor['cpu'] = CPU
    monitor['use_cpu'] = str(USE_CPU)
    monitor['use_temp'] = str(entry.current)
    monitor['memory_total'] = MEMORY_TOTAL // (1024 * 1024)
    monitor['memory_used'] = MEMORY_USED // (1024 * 1024)
    monitor['memory_use'] = str(round(MEMORY_USE, 1))
    monitor['hdd_total'] = HDDTotal
    monitor['hdd_used'] = HDDUsed
    monitor['hdd_use'] = str(round(HDD_USE, 1))

    return monitor


if __name__ == '__main__':
    while 1:
        a = start_client()
        print(a)
        time.sleep(1)
