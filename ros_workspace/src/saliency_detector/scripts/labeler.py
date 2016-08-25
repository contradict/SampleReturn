#!/usr/bin/env python3
import os
import logging
from tempfile import NamedTemporaryFile
from shutil import copy

import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt


LOG=logging.getLogger("Labeler")

class Labeler(object):
    def __init__(self, directory, classes):
        self.directory = directory
        self.classes = classes
        self.figure = plt.figure()
        self.ax_img = self.figure.add_subplot(121)
        self.ax_mask = self.figure.add_subplot(122)
        self.figure.canvas.mpl_connect('key_press_event', self.press)

    def press(self, event):
        LOG.debug('press %s', event.key)
        if event.key in ['%d'%(x+1) for x in range(len(self.classes))]:
            cls = self.classes[int(event.key)-1]
            self.labels[self.current_fname] = cls
            self.title()
            with open(self.outputname, 'a') as of:
                of.write("%s, %s\n"%(self.current_fname, cls))
        elif event.key == "left":
            self.index = max(0, self.index-1)
            self.show()
        elif event.key == "right":
            self.index = min(self.index+1, len(self.filenames))
            self.show()
        elif event.key == "escape" or event.key.upper() == 'Q':
            self.loadlabels(self.outputname)
            self.savelabels(self.outputname)
            plt.close(self.figure)
        LOG.debug('index=%d', self.index)

    def collect_files(self, directory):
        allfiles = os.listdir(self.directory)
        images = filter(lambda x:".png" in x,
                allfiles)
        patches = filter(lambda x:"image.png" in x,
                images)
        return [p for p in patches]

    def label(self, outputname):
        self.filenames = self.collect_files(self.directory)
        self.index = 0
        self.outputname = outputname
        self.loadlabels(self.outputname)
        self.show()

    def loadlabels(self, outputname):
        self.labels = {}
        try:
            with open(outputname, 'r') as of:
                for line in of:
                    fname, label = line.split(',')
                    self.labels[fname.strip()] = label.strip()
        except FileNotFoundError:
            pass

    def savelabels(self, outputname):
        with NamedTemporaryFile('w+') as tf:
            for k, v in sorted(self.labels.items()):
                tf.write("%s, %s\n"%(k,v))
            tf.flush()
            copy(tf.name, outputname)


    def load(self):
        fname = self.filenames[self.index]
        fullname = os.path.join(self.directory, fname)
        stamp, index, _ = fname.split("_")
        maskname = os.path.join(self.directory, "%s_%s_mask.png"%(stamp, index))
        try:
            img = plt.imread(fullname)
            mask = plt.imread(maskname)
            self.current_fname = fname
        except:
            LOG.exception("Unable to open %s", fname)
            return None, None
        return img, mask

    def title(self):
        cls = self.labels.get(self.current_fname, "")
        title = "\n".join((" ".join(self.classes), self.current_fname, cls))
        self.figure.suptitle(title)
        self.figure.canvas.draw()

    def show(self):
        self.ax_img.cla()
        self.ax_mask.cla()
        img, mask = self.load()
        if img is not None and mask is not None:
            self.ax_img.imshow(img)
            self.ax_mask.imshow(mask)
            self.title()
        self.figure.canvas.draw()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--outputname", default="labels.csv",
            help="csv output file name")
    parser.add_argument("--classes", 
            nargs='*',
            help="list of classes")
    parser.add_argument("path",
            help="directory to label")
    options = parser.parse_args()
    logging.basicConfig()
    LOG.setLevel(logging.WARN)
    if options.classes is None:
        options.classes = ['good', 'bad']
    print(options.classes)
    l = Labeler(options.path, options.classes)
    l.label(options.outputname)
    plt.show()

