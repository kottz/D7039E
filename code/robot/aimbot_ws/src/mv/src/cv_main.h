// d7039e.h : Include file for standard system include files,
// or project specific include files.

#include <loguru.cpp>
#include <loguru.hpp>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/objdetect.hpp>
#include <iostream>
#include "vec2.h"
#include "qr_detector.h"
#include "config.h"
//#include "util.h"

using namespace cv;
using namespace std::chrono;
using namespace std;


//void MyLine( Mat img, Point start, Point end );
int line_trace();
Point vecToPoint(vec2f v);
void drawInfo(Mat &img, Point cam, Point track, const vector<decodedObject> &decodedObjects, Rect goal_rect);
//void decode(Mat &im, vector<decodedObject>&decodedObjects)
