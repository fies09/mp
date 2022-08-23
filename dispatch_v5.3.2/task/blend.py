# -*- coding: utf-8 -*-

# ------------------------------------
# Right© of MOONPAC
# Written By lee on 2021.03.26
# Evns: python3 and opencv>=3.0
# ------------------------------------

import cv2
import os
import time

#
def lightBlend(img_address, img_num):
    ''' 该脚本是一个指示灯图片的融合算法
    -Inputs:
            img_address  图片路径
            img_num      待融合的子图数量
    -Return: 
            True or False 状态（融合成功）
    '''
    # Load images
    imgs = []
    img_path = os.path.splitext(img_address)[0]
    for idx in range(img_num):
        path = img_path + '_' + str(idx) + '.jpg'
        img = cv2.imread(path)
        if img is not None:
            imgs.append(img)
        else:
            print('Cannot read image: %s !' % path)
    # Merge
    nums = len(imgs)
    if nums >= 2:
        for i in range(nums - 1):
            if imgs[i].shape != imgs[i + 1].shape:
                print("Warnning: different-sizes of images!")
                w, h, _ = imgs[i].shape
                imgs[i + 1] = cv2.resize(imgs[i + 1], (h, w), interpolation=cv2.INTER_NEAREST)
            binary = cv2.threshold(cv2.subtract(
                cv2.cvtColor(imgs[i], cv2.COLOR_BGR2GRAY),
                cv2.cvtColor(imgs[i + 1], cv2.COLOR_BGR2GRAY)),
                0, 1, cv2.THRESH_BINARY)
            res = cv2.add(cv2.multiply(imgs[i], cv2.cvtColor(binary[1], cv2.COLOR_GRAY2BGR)),
                          cv2.multiply(imgs[i + 1], cv2.cvtColor(1 - binary[1], cv2.COLOR_GRAY2BGR)))
            imgs[i + 1] = res
        mergeAddress = img_path + '_blend.jpg'
        cv2.imwrite(mergeAddress, res)
        os.chmod(mergeAddress, 438)
        return True, mergeAddress
    else:
        print('Cannot blend, image-numbers must be at least 2!')
        return False, None


if __name__ == '__main__':
    # test
    tic = time.time()
    lightBlend('./171416.jpg', 3)
    toc = time.time()
    print('runningTime: {}s'.format(round(toc - tic, 3)))
