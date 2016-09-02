#!/usr/bin/env python
import os
from collections import defaultdict
import json

import cv2
import numpy as np
import matplotlib
matplotlib.use("Qt4Agg")
import matplotlib.pyplot as plt
from tqdm import tqdm

PATH="/media/zlizer/External_Data/patches"
#PATH="/media/russel/aa3337ec-f642-45a5-8709-202851d29f77/SampleReturn/data/color_stats/manipulator_patches"
CSVNAME="alllabels.csv"
#CSVNAME="/media/russel/aa3337ec-f642-45a5-8709-202851d29f77/SampleReturn/data/color_stats/manipulator_labels.csv"
OUTNAME="histdata.csv"
#OUTNAME="manipulator_histdata.csv"

min_saturation=100.
#min_saturation=100./255
N=20
data = defaultdict(list)
with tqdm(total=os.path.getsize(CSVNAME)) as progress:
    with open(CSVNAME) as fn, open(OUTNAME,'a') as of:
        for line in tqdm(fn):
            imgname, label = [x.strip() for x in line.split(',')]
            maskname = imgname.replace("image", "mask")
            img = cv2.imread(os.path.join(PATH,imgname))
            msk = cv2.imread(os.path.join(PATH,maskname))
            msk = cv2.cvtColor(msk, cv2.COLOR_BGR2GRAY)
            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            saturation_mask = cv2.threshold(img_hsv[:,:,1],min_saturation,255,cv2.THRESH_BINARY)[1]
            #saturation_mask = img_hsv[:,:,1]>min_saturation
            mean_saturation = np.mean(img_hsv[:,:,1])
            #mean_saturation = np.mean(img_hsv[:,:,1][msk>0])
            mean_value = np.mean(img_hsv[:,:,2])
            #mean_value = np.mean(img_hsv[:,:,2][msk>0])
            combined_mask = cv2.bitwise_and(saturation_mask, msk)
            #combined_mask = (saturation_mask>0)*(msk>0)
            mean_high_saturation = np.mean(img_hsv[:,:,1][combined_mask>0])
            saturated_pixels = np.sum(combined_mask)/255.
            sat_fraction = np.sum(combined_mask/255.)/np.sum(msk/255.)
            masked_hue = img_hsv[:,:,0][combined_mask>0]
            hue_fraction = np.sum((masked_hue<60) + (masked_hue>240))/np.sum(masked_hue)
            hist, edges = np.histogram(masked_hue, bins=60)
            dominant_hue = np.argmax(hist)*360./60.
            mean_a = np.mean(img_lab[...,1][msk>0])
            mean_b = np.mean(img_lab[...,2][msk>0])
            mean_L = np.mean(img_lab[...,0][msk>0])
            progress.update(len(line))
            of.write("%s, %s, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n"%(
                label, imgname, mean_saturation, mean_high_saturation, mean_value, hue_fraction, dominant_hue,
                saturated_pixels, mean_L, mean_a, mean_b, sat_fraction))


# data[label].append([mean_saturation, mean_value, hue_fraction,
#     dominant_hue, saturated_pixels])
# with open("stats.json", 'w') as fp:
#     json.dump(data, fp, allow_nan=True, sort_keys=True)
#
