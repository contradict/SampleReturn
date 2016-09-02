import sys
cvlibpath="/home/zlizer/src/opencv/build/lib"
#cvlibpath='/usr/local/opencv3/lib/python2.7/dist-packages'
if not cvlibpath in sys.path:
    sys.path.insert(0,cvlibpath)

import numpy as np
import matplotlib
matplotlib.use("Qt4Agg")
import matplotlib.pyplot as plt
import itertools
from plotit import load, make_pd
from sklearn.svm import SVC
from sklearn.cross_validation import train_test_split
from sklearn.preprocessing import StandardScaler
import cv2

lines, pd = load()
noyellow = filter(lambda x: not ('hue' in x[0] and 105<x[9]<125), lines)
pd = make_pd(noyellow)

# manipulator
if False:
    lines, pd = load("manipulator_histdata.csv")
    noyellow = filter(lambda x: not ('hue' in x[0] and 118<x[9]<128), lines)
    pd = make_pd(noyellow)

print "Loaded"

neg_keys = ['none','non','leaf']
pos_keys = ['pre','purplerock','huerock','metalsample','papersample']

neg_feat = None
pos_feat = None

for k in pd.keys():
  if k in neg_keys:
    if neg_feat is None:
      neg_feat = pd[k][:,6:9]
    else:
      neg_feat = np.vstack((neg_feat,pd[k][:,6:9]))

  elif k in pos_keys:
    if pos_feat is None:
      pos_feat = pd[k][:,6:9]
    else:
      pos_feat = np.vstack((pos_feat,pd[k][:,6:9]))

scaler = StandardScaler()
scaler.fit(np.vstack((neg_feat,pos_feat)))
neg_feat = scaler.transform(neg_feat)
pos_feat = scaler.transform(pos_feat)

labels = np.hstack((-np.ones(neg_feat.shape[0]),np.ones(pos_feat.shape[0])))

feat_train, feat_test, labels_train, labels_test = \
    train_test_split(np.vstack((neg_feat, pos_feat)),labels, test_size=0.2)

svm = SVC(gamma=1.0, C=1.0);
svm.fit(feat_train, labels_train)
print "Train error rate", len(np.where(svm.predict(feat_train) != labels_train)[0])/float(len(labels_train))
print "Test error rate", len(np.where(svm.predict(feat_test) != labels_test)[0])/float(len(labels_test))

cv_svm = cv2.ml.SVM_create()
cv_svm.train(feat_train.astype(np.float32), cv2.ml.ROW_SAMPLE,labels_train.astype(np.int))
print "Opencv train error rate", len(np.where(labels_train != \
    cv_svm.predict(feat_train.astype(np.float32))[1].flatten())[0])/float(len(labels_train))
print "Opencv test error rate", len(np.where(labels_test != \
    cv_svm.predict(feat_test.astype(np.float32))[1].flatten())[0])/float(len(labels_test))

def draw_svm(lrange, arange, brange, svm, pd):
    keys = pd.keys()
    cm = plt.get_cmap('spectral')
    labelcolors = dict([(l, cm(1.*i/len(keys))) for i,l in enumerate(keys)])
    for i,L in enumerate(lrange):
        aa, bb = np.meshgrid(arange, brange)
        plt.subplot(len(lrange), 1, i+1)
        gpts = np.c_[ L*np.ones_like(aa).ravel(), aa.ravel(), bb.ravel()]
        sgpts = scaler.transform(gpts)
        if type(svm) == SVC:
            Z = svm.decision_function(sgpts)
        else:
            Z = svm.predict(sgpts.astype(np.float32), None, cv2.ml.StatModel_RAW_OUTPUT)[1].flatten()
        Z = Z.reshape(aa.shape)
        plt.contourf(aa, bb, Z)
        plt.colorbar()
        for k in pd.keys():
            pts = np.hstack([pd[k][:,6:7], pd[k][:,7:8], pd[k][:,8:9]])
            pts = pts[((L-25)<pts[:,0])*(pts[:,0]<(L+25))]
            plt.scatter(pts[:,1],
                        pts[:,2],
                        label=k,
                        c=labelcolors[k])
        plt.xlabel("Mean A")
        plt.ylabel("Mean B")
        plt.title("L=%f"%L)
        plt.legend()

plt.figure(4)
plt.clf()
#draw_svm([100,150,200], np.linspace(50,200,500), np.linspace(50,200,500), svm, pd)
draw_svm([100,150,200], np.linspace(50,200,500), np.linspace(50,200,500), cv_svm, pd)
plt.draw()


def save(fname):
    cv_svm.save(fname)
    fs = cv2.FileStorage(fname, cv2.FILE_STORAGE_APPEND)
    fs.write("mean", scaler.mean_.reshape((1,3)).astype('f4'))
    fs.write("std", scaler.std_.reshape((1,3)).astype('f4'))
    fs.release()
