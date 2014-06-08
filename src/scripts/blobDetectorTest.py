import cv2

def getBlackOnWhiteDetector():
    blobParams = cv2.SimpleBlobDetector_Params()
    blobParams.blobColor = 0
    blobParams.minArea = 50
    blobParams.maxArea = 5000
    blobParams.minThreshold = 20
    blobParams.maxThreshold = 220
    blobParams.thresholdStep = 10
    blobParams.minDistBetweenBlobs = 2.0
    blobParams.minRepeatability = 3
    blobParams.filterByColor = True
    blobParams.filterByArea = True
    blobParams.filterByCircularity = False
    blobParams.filterByInertia = False
    blobParams.filterByConvexity = True

    return cv2.SimpleBlobDetector(blobParams)

def getWhiteOnWhiteDetector():
    blobParams = cv2.SimpleBlobDetector_Params()
    blobParams.blobColor = 255
    blobParams.minArea = 50
    blobParams.maxArea = 5000
    blobParams.minThreshold = 20
    blobParams.maxThreshold = 220
    blobParams.thresholdStep = 10
    blobParams.minDistBetweenBlobs = 2.0
    blobParams.minRepeatability = 3
    blobParams.filterByColor = True
    blobParams.filterByArea = True
    blobParams.filterByCircularity = False
    blobParams.filterByInertia = False
    blobParams.filterByConvexity = True

    return cv2.SimpleBlobDetector(blobParams)

def testForSideMarker(imageData, blobDetector):
    retval = True
    blobs = blobDetector.detect(imageData)
    if len(blobs) < 3:
        print 'not enough blobs!'
        return False
    # sort by size, largest first
    sortedBlobs = sorted(blobs, key=lambda b:b.size, reverse=True)

    # sometimes we see a huge blob that's not related, so loop until we're
    # out of blobs to test
    while(len(sortedBlobs) >= 3):
        # reset the flag so we can look at the value at the end of the loop
        retval = True

        topThree = sortedBlobs[:3]
        # sort by y coordinate to get them stacked (they should be vertical)
        topThree = sorted(topThree, key=lambda b:b.pt[1])
        avgSize = 0.0
        for b in topThree:
            avgSize += b.size
        avgSize /= 3
        print '\t\tavg size: %f' % (avgSize)
        print '\t\t0 size: %f' % (topThree[0].size)
        print '\t\t1 size: %f' % (topThree[1].size)
        print '\t\t2 size: %f' % (topThree[2].size)

        # guess. should be a parameter
        maxSizeError = 0.1

        for i in range(len(topThree)):
            b = topThree[i]
            if (b.size < avgSize * (1.0 - maxSizeError)) or\
                    (b.size > avgSize * (1.0 + maxSizeError)):
                print '\t\tkeypoint %d has size outside of %f error' % (i, maxSizeError)
                retval = False

        horizontal0to1 = topThree[0].pt[0]-topThree[1].pt[0]
        horizontal1to2 = topThree[1].pt[0]-topThree[2].pt[0]
        print '\t\t0-1 horizontal error: %f' % (horizontal0to1)
        print '\t\t1-2 horizontal error: %f' % (horizontal1to2)

        distance0to1 = sqrt((topThree[0].pt[0]-topThree[1].pt[0])**2 + (topThree[0].pt[1]-topThree[1].pt[1])**2)
        distance1to2 = sqrt((topThree[1].pt[0]-topThree[2].pt[0])**2 + (topThree[1].pt[1]-topThree[2].pt[1])**2)
        print '\t\t0-1 distance: %f' % (distance0to1)
        print '\t\t1-2 distance: %f' % (distance1to2)

        # another guess, should be a parameter
        maxDistanceError = 0.05
        
        if abs(distance0to1 - distance1to2) > min(distance0to1, distance1to2) * maxDistanceError:
            print '\t\tdistance error too high!'
            retval = False

        # one more should be a parameter
        maxHorizontalError = 0.1

        if abs(horizontal0to1 - horizontal1to2) > abs(min(horizontal0to1, horizontal1to2)) * maxHorizontalError:
            print '\t\thorizontal error too different'
            retval = False

        # if we made it here and retval is still true, break,
        # we must have found it
        if(retval):
            break
        else:
            # if we didn't find it, remove one element from sorted blobs and
            # try again.
            sortedBlobs = sortedBlobs[1:]

    return retval

def testImage(imageData):
    retval = False
    print '\tlooking for black on white...'
    if(testForSideMarker(imageData, blackOnWhiteBlobDetector)):
        print '\tfound black on white!'
        retval = True
    else:
        print '\tdid not find black on white'
    print '\tlooking for white on black...'
    if(testForSideMarker(imageData, whiteOnBlackBlobDetector)):
        print '\tfound white on black!'
        retval = True
    else:
        print '\tdid not find white on black'

    print '\tlooking for black on white circles grid...'
    found, centers = cv2.findCirclesGrid(imageData,
            (3, 9),
            flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
            blobDetector=blackOnWhiteBlobDetector)
    if found:
        print '\tfound black on white circles grid!'
        retval = True

    print '\tlooking for white on black circles grid...'
    found, centers = cv2.findCirclesGrid(imageData,
            (3, 9),
            flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
            blobDetector=whiteOnBlackBlobDetector)
    if found:
        print '\tfound white on black circles grid!'
        retval = True

    return retval


