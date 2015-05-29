import numpy as np
import cv2
import sys
from matplotlib.pyplot import imshow,draw
import scipy.linalg
import time

#img = cv2.imread(sys.argv[1])
#img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
#hsv = cv2.cvtColor(img,cv2.COLOR_RGB2HSV)
#lab = cv2.cvtColor(img,cv2.COLOR_RGB2LAB)
#
#hsv_map = np.zeros((180, 256, 3), np.uint8)
#h, s = np.indices(hsv_map.shape[:2])
#hsv_map[:,:,0] = h
#hsv_map[:,:,1] = s
#hsv_map[:,:,2] = 255
#hsv_map = cv2.cvtColor(hsv_map, cv2.COLOR_HSV2RGB)
#
#hist = cv2.calcHist([img],[0,1,2],None,[12,12,12],[0,255,0,255,0,255])
#hist /= scipy.linalg.norm(hist)

# Load no-sample image or images, trim sky, compute RGB hist
# From metal_sample: frame 23, 25, 30, 36, 40, 47, 48
print len(sys.argv)
if (len(sys.argv)==3):
    acc_hist = None
    for num in ['23','25','30','36','40','47','48']:
        train_img = cv2.cvtColor(cv2.imread(sys.argv[1]+num+'.jpg'),cv2.COLOR_BGR2RGB)
        trim_train_img = train_img[600:,...].copy()
        hist = cv2.calcHist([trim_train_img],[0,1,2],None,[12,12,12],[0,256,0,256,0,256])
        if acc_hist == None:
            acc_hist = hist
        else:
            acc_hist += hist
    # Normalize hist
    hist = acc_hist/np.sum(acc_hist)
elif (len(sys.argv)==2):
    train_img = cv2.cvtColor(cv2.imread(sys.argv[1]),cv2.COLOR_BGR2RGB)
    trim_train_img = train_img[600:,...].copy()
    trim_train_img = cv2.resize(trim_train_img,(1000,540),interpolation=cv2.INTER_AREA)
    print "Calc Hist Start: ",time.clock()
    hist = cv2.calcHist([trim_train_img],[0,1,2],None,[12,12,12],[0,256,0,256,0,256])
    # Normalize hist
    hist = hist/np.sum(hist)

# Reduce hist to n ~= 100 bins (try to cover some fraction of pixels. e.g.95%)
flat_hist = hist.flatten()
top_bins = None
bin_width = 256/12.
for i in range(40):
    arg = np.argsort(flat_hist)[-(i+1)]
    index = np.unravel_index(arg,(12,12,12))
    if top_bins == None:
        top_bins = np.array([index[0]*bin_width+bin_width/2.,
                            index[1]*bin_width+bin_width/2.,
                            index[2]*bin_width+bin_width/2.,
                            flat_hist[arg]])
    else:
        bins = np.array([index[0]*bin_width+bin_width/2.,
                            index[1]*bin_width+bin_width/2.,
                            index[2]*bin_width+bin_width/2.,
                            flat_hist[arg]])
        top_bins = np.vstack((top_bins,bins))
# Turn top_bins into Lab space for distance metric
for i in range(top_bins.shape[0]):
    lab_bin = cv2.cvtColor(top_bins[i,:3].reshape((1,1,3)).astype(np.uint8),cv2.COLOR_RGB2LAB)
    hsv_bin = cv2.cvtColor(top_bins[i,:3].reshape((1,1,3)).astype(np.uint8),cv2.COLOR_RGB2HSV)
    #top_bins[i,:3] = lab_bin.reshape((3))
    top_bins[i,1:3] = lab_bin.reshape((3))[1:3]
    top_bins[i,0] = hsv_bin.reshape((3))[0]
print "Calc Hist End: ",time.clock()
# For query image, iterate over hist bins to compute per-pixel score,
# which is distance between query pixel and hist bin color * hist bin count
if (len(sys.argv)==3):
    query_img = cv2.cvtColor(cv2.imread(sys.argv[2]),cv2.COLOR_BGR2LAB)
elif (len(sys.argv)==2):
    #query_img_lab = cv2.cvtColor(cv2.imread(sys.argv[1]),cv2.COLOR_BGR2LAB)
    #query_img_hsv = cv2.cvtColor(cv2.imread(sys.argv[1]),cv2.COLOR_BGR2HSV)
    query_img_lab = cv2.cvtColor(trim_train_img,cv2.COLOR_RGB2LAB)
    query_img_hsv = cv2.cvtColor(trim_train_img,cv2.COLOR_RGB2HSV)
    query_img_lab[...,0] = query_img_hsv[...,0]
print "Img Comp Start: ",time.clock()
#trim_query_img = query_img_lab[600:,...].copy()
#trim_query_img = cv2.resize(trim_query_img,(1000,540))
trim_query_img = query_img_lab.copy()
score_img = np.zeros((trim_query_img.shape[0],trim_query_img.shape[1]),np.float32)
for n in range(top_bins.shape[0]):
    dist = np.abs(trim_query_img - top_bins[n,:3])
    #diff_img = trim_query_img[...,1:] - top_bins[n,1:3]
    #score_img += np.hypot(diff_img[...,0],diff_img[...,1])*top_bins[n,3]
    score_img += np.sum(dist,axis=2)*top_bins[n,3]
print "Img Comp End: ",time.clock()

# Return image

#h = cv2.calcHist( [hsv], [0, 1], None, [180, 256], [0, 180, 0, 256] )

#hist_scale = float(sys.argv[2])

#h = np.clip(h*0.005*hist_scale, 0, 1)
#vis = hsv_map*h[:,:,np.newaxis] / 255.0

#hist = cv2.calcHist([lab[700:,...]],[0,1,2],None,[256,256,256],[0,255,0,255,0,255])
#log_hist = np.log10(hist + 1)

#def compute_prob_img(img, hist):
#    prob_img = np.zeros((img.shape[0],img.shape[1]),np.float32)
#    for i in range(img.shape[0]):
#        print i
#        for j in range(img.shape[1]):
#            score = 0
#            for x in range(hist.shape[0]):
#                for y in range(hist.shape[1]):
#                    for z in range(hist.shape[2]):
#                        prob = hist[x,y,z]
#                        score += prob*scipy.linalg.norm(img[i,j]-(10.6667*np.array([x,y,z])))
#                        #r_index = np.floor(img[i,j,0]/x)
#                        #g_index = np.floor(img[i,j,1]/y)
#                        #b_index = np.floor(img[i,j,2]/z)
#            prob_img[i,j] = score
#            #for k in range(12):
#            #    hist_color = np.array([np.floor
#            #    prob_img[i,j] += hist[r_index,g_index,b_index]
#            #prob_img[i,j] = hist[img[i,j][0],img[i,j][1],img[i,j][2]]
#            #prob_img[i,j] = log_hist[lab[i,j][1],lab[i,j][2]]
#    return prob_img

#prob_img = compute_prob_img(img,hist)
