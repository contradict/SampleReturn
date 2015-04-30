import numpy as np
import cv2
import sys

import test_shape_metric

pos_results = test_shape_metric.compute_shape_metrics(sys.argv[1])
neg_results = test_shape_metric.compute_shape_metrics(sys.argv[2])

if (sys.argv[3]=="True"):
    # Normalize
    #norm_pos_results = (pos_results-np.mean(pos_results,0))/(2*np.std(pos_results,0))
    #norm_neg_results = (neg_results-np.mean(neg_results,0))/(2*np.std(neg_results,0))

    training_data = np.float32(np.vstack((norm_pos_results[:15],norm_neg_results[:15])))
    training_labels = np.float32(np.hstack((np.ones(15),-np.ones(15))))

    test_data = np.float32(np.vstack((norm_pos_results[15:],norm_neg_results[15:])))
    test_labels = np.float32(np.hstack((np.ones_like(pos_results[15:,0]),-np.ones_like(neg_results[15:,0]))))
else:
    training_data = np.float32(np.vstack((pos_results[:15],neg_results[:15])))
    training_labels = np.float32(np.hstack((np.ones(15),-np.ones(15))))

    test_data = np.float32(np.vstack((pos_results[15:],neg_results[15:])))
    test_labels = np.float32(np.hstack((np.ones_like(pos_results[15:,0]),-np.ones_like(neg_results[15:,0]))))

errors = []
recalls = []
for g in [-5,-4,-3,-2,-1,0,1,2,3,4,5]:
  for c in [-5,-4,-3,-2,-1,0,1,2,3,4,5]:
    test_params = dict(kernel_type=cv2.SVM_RBF,svm_type=cv2.SVM_C_SVC,gamma=10**g,C=10**c)
    svm = cv2.SVM()
    #svm = cv2.SVM(training_data,training_labels,params=params)
    svm.train(training_data,training_labels,params=test_params)
    recall = 0.0
    error = 0.0
    for i in range(test_data.shape[0]):
      res = svm.predict(test_data[i])
      if res != test_labels[i]:
        error += 1.0
    print "Gamma:",params['gamma'],"C:",params['C']
    print "Error rate: ",(error/test_data.shape[0])
    for i in range(training_data.shape[0]):
      res = svm.predict(training_data[i])
      if res == training_labels[i]:
        recall += 1.0
    print "Recall:",(recall/training_data.shape[0])
    errors.append(error/test_data.shape[0])
    recalls.append(recall/training_data.shape[0])
