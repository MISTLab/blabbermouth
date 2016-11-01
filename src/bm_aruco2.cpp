#include <string>
#include <iostream>
#include "bm_aruco2.h"

pair< double, double > AvrgTime(0, 0); // determines the average time required for detection

cv::Mat resize(const cv::Mat &in,int width){
    if (in.size().width<=width) return in;
    float yf=float(  width)/float(in.size().width);
    cv::Mat im2;
    cv::resize(in,im2,cv::Size(width,float(in.size().height)*yf));
    return im2;
    
}

int getposes(VideoCapture TheVideoCapturer, CameraParameters TheCameraParameters, float TheMarkerSize, pose2d *p2d, int *nTags, int debugwin) {
    MarkerDetector MDetector;
    Mat TheInputImage, TheInputImageCopy;
    Mat rotation;float angle;
    vector< Marker > TheMarkers;
    MDetector.setDictionary(Dictionary::getTypeFromString("ARUCO"));//sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
    MDetector.setThresholdParams(7, 7);
    MDetector.setThresholdParamRange(2, 0);

    try {
        
        //gui requirements : the trackbars to change this parameters
        if(debugwin)    cv::namedWindow("in");
        
            
            TheVideoCapturer.retrieve(TheInputImage);
            // copy image
            double tick = (double)getTickCount(); // for checking the speed
            // Detection of markers in the image passed
            TheMarkers= MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);
            // chekc the speed by calculating the mean speed of all iterations
            AvrgTime.first += ((double)getTickCount() - tick) / getTickFrequency();
            AvrgTime.second++;
            cout << "Time detection=" << 1000 * AvrgTime.first / AvrgTime.second << " milliseconds nmarkers=" << TheMarkers.size() << std::endl;
            
            // print marker info and draw the markers in image
            TheInputImage.copyTo(TheInputImageCopy);
            
            *nTags = TheMarkers.size();
            for (unsigned int i = 0; i < TheMarkers.size(); i++) {
                Rodrigues(TheMarkers[i].Rvec, rotation);
                angle = atan2(-rotation.at<float>(0, 1), rotation.at<float>(0, 0));
                if(TheMarkers[i].id<=10){    //tags between 1 and 10.
                    p2d[TheMarkers[i].id-1].x=TheMarkers[i].Tvec.at<float>(0);
                    p2d[TheMarkers[i].id-1].y=TheMarkers[i].Tvec.at<float>(1);
                    p2d[TheMarkers[i].id-1].theta=angle;
//                    cout << "\r [Aruco] marker id= " << p2d[TheMarkers[i].id-1].idr << " : " << p2d[TheMarkers[i].id-1].x << " m, " << p2d[TheMarkers[i].id-1].y << " m, " << p2d[TheMarkers[i].id-1].theta << " rad" << endl;
                }
                if(debugwin)
                {
                    TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255));
                    if (TheCameraParameters.isValid()){
                        CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
                        CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
                    }
                }
            }
        
        // show input with augmented information and  the thresholded image
        if(debugwin)    cv::imshow("in", resize(TheInputImageCopy,1280));
        
    } catch (std::exception &ex)
    
    {
        cout << "Exception :" << ex.what() << endl;
    }
    return 1;
}

