import numpy as np
import cv2
import sys
import matplotlib.pyplot
import os, glob

# This is going to load the image directories of each object, extract the largest MSER region,
# compute the color histograms, and generate the mean and covariance of each.

#red_puck
#orange_pipe
#pre_cached
#pink_sphere

objects_path = "/home/zlizer/Desktop/ColorModelImages/"

#objects = ('pre_cached/','red_puck/','orange_pipe/','pink_sphere/')
objects = ('platform/','pre_cached/','pink_sphere/')
#objects = ('red_puck/')

if __name__ == '__main__':
  if sys.argv[1] == 'pixel':
    for ob in objects:
      objacc = np.array([0,0,0])
      objcount = 0
      objmeans = None
      imglist = os.listdir(objects_path+ob)
      for image in imglist:
        print objects_path+ob+image
        if image == 'hists':
          continue
        img = cv2.imread(objects_path+ob+image)
        print img.shape
        if ob == 'red_puck/' or ob == 'orange_pipe/' or ob == 'platform/':
          lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
          a = lab[:,:,1].copy()
        if ob == 'pre_cached/' or ob == 'pink_sphere/':
          lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
          a = lab[:,:,2].copy()
        mser = cv2.MSER(5,2000,244000,0.25,0.2,200,1.01,0.003,5)
        vis = img.copy()
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        select = img.copy()

        regions = mser.detect(a, None)
        hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in regions]
        #print hulls
        cv2.polylines(vis, hulls, 1, (0, 255, 0), 3)
        for hull in hulls:
          r = cv2.boundingRect(hull)
          cv2.rectangle(vis,(r[0],r[1]),(r[0]+r[2],r[1]+r[3]),(0,0,255),2)

        cv2.namedWindow('img',cv2.cv.CV_WINDOW_NORMAL)
        cv2.imshow('img', vis)
        cv2.namedWindow('lab',cv2.cv.CV_WINDOW_NORMAL)
        cv2.imshow('lab',lab)
        cv2.namedWindow('A',cv2.cv.CV_WINDOW_NORMAL)
        cv2.imshow('A',a)
        i = 0
        while True:
          cv2.namedWindow('select',cv2.cv.CV_WINDOW_NORMAL)
          if len(hulls) == 0:
            break
          else:
            cv2.polylines(select,[hulls[i]],1,(0,255,0),5)
            r = cv2.boundingRect(hulls[i])
            cv2.rectangle(select,(r[0],r[1]),(r[0]+r[2],r[1]+r[3]),(0,0,255),4)
            cv2.imshow('select',select)
            key = cv2.waitKey(5)
            if key == 1113939:
              if i < len(hulls)-1:
                print "Forward"
                i += 1
                print i
              else:
                print "At last hull"
              select = img.copy()
              cv2.polylines(select,[hulls[i]],1,(0,255,0),5)
              r = cv2.boundingRect(hulls[i])
              cv2.rectangle(select,(r[0],r[1]),(r[0]+r[2],r[1]+r[3]),(0,0,255),4)
              cv2.imshow('select',select)
            if key == 1113937:
              if i > 0:
                print "Back"
                i -= 1
              else:
                print "At first hull"
              select = img.copy()
              cv2.polylines(select,[hulls[i]],1,(0,255,0),5)
              r = cv2.boundingRect(hulls[i])
              cv2.rectangle(select,(r[0],r[1]),(r[0]+r[2],r[1]+r[3]),(0,0,255),4)
              cv2.imshow('select',select)
            if key == 1048603:
              break
            if key == 1113940:
              acc = np.array([0,0,0])
              count = 0
              for j in range(r[3]):
                for k in range(r[2]):
                  if cv2.pointPolygonTest(hulls[i],(j,k),False):
                    #print lab[r[1]+j,r[0]+k,:]
                    acc += lab[r[1]+j,r[0]+k,:]
                    count += 1
              print count,acc,(acc/count)
              objacc += (acc/count)
              objcount += 1
              if objmeans == None:
                objmeans = (acc/count)
              else:
                objmeans = np.vstack((objmeans,(acc/count)))
              print objacc,objcount,(objacc/objcount)
              print objmeans
              #np.savez(objects_path+ob+'/hists/'+image,hist=colorhist[0],i=i)
              print "Tagged"
      np.savez(objects_path+ob+'colormeans',objmeans=objmeans,objacc=objacc,count=objcount)

  else:
    for ob in objects:
      imglist = os.listdir(objects_path+ob)
      for image in imglist:
        print objects_path+ob+image
        if image == 'hists':
          continue
        img = cv2.imread(objects_path+ob+image)
        print img.shape
        if ob == 'red_puck/' or ob == 'orange_pipe/':
          lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
          a = lab[:,:,1].copy()
        if ob == 'pre_cached/' or ob == 'pink_sphere/':
          lab = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
          a = lab.copy()
        bins = 10
        mser = cv2.MSER(5,2000,144000,0.25,0.2,200,1.01,0.003,5)
        vis = img.copy()
        select = img.copy()

        regions = mser.detect(a, None)
        hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in regions]
        print hulls
        cv2.polylines(vis, hulls, 1, (0, 255, 0), 3)
        for hull in hulls:
          r = cv2.boundingRect(hull)
          cv2.rectangle(vis,(r[0],r[1]),(r[0]+r[2],r[1]+r[3]),(0,0,255),2)
          weights = np.zeros((r[3],r[2]))
          for i in range(r[3]):
            for j in range(r[2]):
              if cv2.pointPolygonTest(hull,(i,j),False):
                weights[i,j] = 1
          colorhist = np.histogram(a[r[1]:r[1]+r[3],r[0]:r[0]+r[2]],bins,range=(0,255),weights=weights,normed=True)

      cv2.namedWindow('img',cv2.cv.CV_WINDOW_NORMAL)
      cv2.imshow('img', vis)
      cv2.namedWindow('lab',cv2.cv.CV_WINDOW_NORMAL)
      cv2.imshow('lab',lab)
      cv2.namedWindow('A',cv2.cv.CV_WINDOW_NORMAL)
      cv2.imshow('A',a)
      i = 0
      while True:
        cv2.namedWindow('select',cv2.cv.CV_WINDOW_NORMAL)
        if len(hulls) == 0:
          break
        else:
          cv2.polylines(select,[hulls[i]],1,(0,255,0),5)
          r = cv2.boundingRect(hulls[i])
          cv2.rectangle(select,(r[0],r[1]),(r[0]+r[2],r[1]+r[3]),(0,0,255),4)
          cv2.imshow('select',select)
          key = cv2.waitKey(5)
          if key == 1113939:
            if i < len(hulls)-1:
              print "Forward"
              i += 1
              print i
            else:
              print "At last hull"
            select = img.copy()
            cv2.polylines(select,[hulls[i]],1,(0,255,0),5)
            r = cv2.boundingRect(hulls[i])
            cv2.rectangle(select,(r[0],r[1]),(r[0]+r[2],r[1]+r[3]),(0,0,255),4)
            cv2.imshow('select',select)
          if key == 1113937:
            if i > 0:
              print "Back"
              i -= 1
            else:
              print "At first hull"
            select = img.copy()
            cv2.polylines(select,[hulls[i]],1,(0,255,0),5)
            r = cv2.boundingRect(hulls[i])
            cv2.rectangle(select,(r[0],r[1]),(r[0]+r[2],r[1]+r[3]),(0,0,255),4)
            cv2.imshow('select',select)
          if key == 1048603:
            break
          if key == 1113940:
            weights = np.zeros((r[3],r[2]))
            for j in range(r[3]):
              for k in range(r[2]):
                if cv2.pointPolygonTest(hulls[i],(j,k),False):
                  weights[j,k] = 1
            colorhist = np.histogram(a[r[1]:r[1]+r[3],r[0]:r[0]+r[2]],bins,range=(0,255),weights=weights,normed=True)
            print colorhist
            np.savez(objects_path+ob+'/hists/'+image,hist=colorhist[0],i=i)
            print "Tagged"
