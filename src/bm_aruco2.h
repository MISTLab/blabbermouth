#ifndef BM_ARUCO_H
#define BM_ARUCO_H


#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include "bm_pose2d.h"

using namespace cv;
using namespace aruco;

int getposes(VideoCapture TheVideoCapturer, CameraParameters TheCameraParameters, float TheMarkerSize, pose2d *p2d, int *nTags, int debugwin);

#endif
