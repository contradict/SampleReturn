import numpy as np
import cv2
import os
import sys
import csv

def compute_shape_metrics(labels):
    #reader = csv.reader(open(sys.argv[1]))
    reader = csv.reader(open(labels))

    results = None

    for row in reader:
      if row[0] == "IMG":
        continue
      name = row[0]
      if (row[1] == " metal_lines" or row[1]==" metal_tree"):
        continue
      buf = 10
      x = int(row[2])
      y = int(row[3])
      x_size = int(row[4])/2 + buf
      y_size = int(row[5])/2 + buf

      # Load Images
      img = cv2.imread(name+".jpg")
      # Color Space Conversion
      lab = cv2.cvtColor(img,cv2.COLOR_BGR2LAB)
      win = img[y-y_size:y+y_size,x-x_size:x+x_size]
      lab_win = lab[y-y_size:y+y_size,x-x_size:x+x_size]
      #cv2.imshow("test",lab_win[...,1])
      #cv2.waitKey(-1)
      mid = (float(np.max(lab_win[...,1]))+float(np.min(lab_win[...,1])))/2.
      #print mid, np.max(lab_win[...,1]), np.min(lab_win[...,1]), lab_win.dtype
      thresh = cv2.threshold(lab_win[...,1],mid,255,cv2.THRESH_BINARY)[1]
      # Contours
      contours, hier = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
      largest_area = 0
      largest_contour = 0
      for i in contours:
          area = cv2.contourArea(i)
          if area > largest_area:
              largest_area = area
              largest_contour = i
      # Hulls
      hull = cv2.convexHull(largest_contour,returnPoints=False)
      hull_pts = cv2.convexHull(largest_contour)
      # Perimeter Difference
      perimeter_ratio = cv2.arcLength(hull_pts,True)/cv2.arcLength(largest_contour,True)
      # Area Difference
      area_ratio = cv2.contourArea(hull_pts)/cv2.contourArea(largest_contour)
      # Max Convexity Defect
      defects = cv2.convexityDefects(largest_contour,hull)
      # Combined Score
      print perimeter_ratio,area_ratio,np.max(defects[...,-1]),np.sum(defects[...,-1]),row[-1],row[1]
      if results == None:
          results = np.array([perimeter_ratio,area_ratio])#,np.max(defects[...,-1]),np.sum(defects[...,-1])])
      else:
          results = np.vstack((results,np.array([perimeter_ratio,area_ratio])))#,np.max(defects[...,-1]),np.sum(defects[...,-1])])))
    return results


if __name__=="__main__":
    compute_shape_metrics(sys.argv[1])
