#include "cv_main.h"
#include <chrono>
using namespace std::chrono; 

using namespace std;
using namespace cv;
using namespace zbar;

//vector<mask> get_mask_vector()

Mat create_color_mask(Mat &img, vector<mask> &mask_vec) {
	int sizeX = img.cols;
	int sizeY = img.rows;

	int vertical_size = 10;

	Rect mask_area(0, (sizeY-vertical_size)/2, sizeX, vertical_size);
	Mat cropped = img(mask_area);
	Mat hsv;
	cvtColor(cropped, hsv, COLOR_BGR2HSV);

	Mat color_range[mask_vec.size()];

	for(auto i = 0; i < mask_vec.size(); ++i ) {
		inRange(hsv, mask_vec[i].low, mask_vec[i].high, color_range[i]);
	}

	for(auto i = 0; i < mask_vec.size()-1; ++i ) {
		bitwise_or(color_range[i], color_range[i+1], color_range[i+1]);
	}
	Mat mask = color_range[mask_vec.size()-1];

	return mask;
}

int line_trace_mask(Mat &mask) {
	int cX;
	Moments m = moments(mask);
	if(m.m00 == 0)
		cX = 0;
	else
		cX = m.m10 / m.m00;

	return cX;
}

float angle(Point p1, Point p2) {
	vec2f p1v = vec2f(p1.x, p1.y);
	vec2f p2v = vec2f(p2.x, p2.y);
	vec2f u = p1v - p2v;
	float angle = u.argument();

	return angle;
}

Point trace_line_and_qr(Mat &img, vector<mask> &mask_vec, QRDetector &qr_detector, vector<decodedObject> &decodedObjects, bool &qr_tracked) {
	int sizeX = img.cols;
	int sizeY = img.rows;

	//LINE TRACE
	Mat mask = create_color_mask(img, mask_vec);
	auto x = line_trace_mask(mask);

	Point track_point = Point(x, sizeY/2);

	//QR DETECT
	bool detected = qr_detector.Detect(img, &decodedObjects);
	
	qr_tracked = false;
	//Pick qr location over line location if it is in the upper half of the image
	if(detected) {
		Point center = QRDetector::GetCenter(decodedObjects[0]);
		if(center.y < sizeY/2) {
			track_point = center;
			qr_tracked = true;
		}
	}
	return track_point;
}

bool follow_line_until_qr(VideoCapture &cap, vector<mask> &mask_vec, QRDetector &qr_detector, Rect goal_rect, bool draw_gui) {

	while (true) {
		auto start = high_resolution_clock::now();
		Mat frame1;
		cap >> frame1;
		Mat frame;


		if (frame1.empty()) {
			LOG_F(INFO, "Empty frame");
			break;
		}
		resize(frame1, frame, Size(frame1.cols * 0.5, frame1.rows * 0.5), 0, 0, INTER_LINEAR);
		
		vector<decodedObject> decodedObjects;
		bool qr_tracked;
		Point track_point = trace_line_and_qr(frame, mask_vec, qr_detector, decodedObjects, qr_tracked);
		//Break when QR is in correct position

		if(qr_tracked && goal_rect.contains(track_point)) {
			LOG_F(INFO, "QR is in correct position. Stopping Robot.");
			return true;
		}

		if(draw_gui) {
			int sizeX = frame.cols;
			int sizeY = frame.rows;
			int camX = sizeX / 2;
			int camY = sizeY - 0;
			Mat mask = create_color_mask(frame, mask_vec);
			drawInfo(frame, Point(camX, camY), track_point, decodedObjects, goal_rect);
			imshow("frame", frame);
			imshow("mask", mask);
			
			
			char c = (char)waitKey(25);
			if (c == 27)
				break;
		}

		//video.write(frame);
		auto stop = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(stop - start);
		//
		cout << "dur: " << duration.count() << endl;
	}
	return false;
}

int process_video() {
	Config config = Config::GetConfigFromFile("config.toml");
	bool draw_gui = config.DrawGUI();
	vector<mask> mask_vec = config.GetMask();
	VideoCapture cap(config.GetVideoSource());
	if (!cap.isOpened()) {
		LOG_F(ERROR, "Could not open video source");
		return -1;
	}
	//cap.set(CAP_PROP_FRAME_WIDTH,256);
	//cap.set(CAP_PROP_FRAME_HEIGHT,144);

	//writer
	//int frame_width = cap.get(CAP_PROP_FRAME_WIDTH);
	//int frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);
	//VideoWriter video("out_mov.avi", VideoWriter::fourcc('M','J','P','G'),30,Size(frame_width, frame_height));

	//lower mask

	
	QRDetector qr_detector = QRDetector();
	Rect goal_rect = config.GetGoalRect();
	follow_line_until_qr(cap, mask_vec, qr_detector, goal_rect, draw_gui);

	LOG_F(INFO, "Waiting for next command.");

	cap.release();
	//video.release();
	destroyAllWindows();
	
	return 0;
}

Point vecToPoint(vec2f v) {
	int x = (int)v.x;
	int y = (int)v.y;
	return Point(x,y);
}

void drawInfo(Mat &img, Point cam, Point track, const vector<decodedObject> &decodedObjects, Rect goal_rect) {

	line(img, track, cam, Scalar(255,0,0),2);

	int radius = 80;
	vec2f camv = vec2f((float)cam.x, (float)cam.y);
	vec2f trackv = vec2f((float)track.x, (float)track.y);
	vec2f u = trackv - camv;
	u = u.normalize() * radius;
	float angle = u.argument();
	//cout << angle << endl;
	rectangle(img, goal_rect, Scalar(0,0,255),1,8,0);
	line(img, vecToPoint(camv), vecToPoint(camv+u), Scalar(0,0,0), 5);
	ellipse(img, vecToPoint(camv), Size(u.length(), u.length()), 0, 0, -angle, Scalar(0,0,0), 5);
	vec2f textOffset = vec2f(radius+10,-10);
	putText(img, to_string((int)angle), vecToPoint(camv+textOffset), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);

	
	if(decodedObjects.size() > 0) {
		//cout << "detected" << endl;
		//cout << decodedObjects[0].data << endl;
		//draw lines around qr code
		Point center = QRDetector::GetCenter(decodedObjects[0]);
		circle(img, center,10, Scalar(255,255,255),-1, 8,0);
		line(img, decodedObjects[0].location[0], decodedObjects[0].location[1], Scalar(255,100,0),3);
		line(img, decodedObjects[0].location[1], decodedObjects[0].location[2], Scalar(255,200,0),3);
		line(img, decodedObjects[0].location[2], decodedObjects[0].location[3], Scalar(255,0,100),3);
		line(img, decodedObjects[0].location[3], decodedObjects[0].location[0], Scalar(255,0,200),3);
		putText(img, decodedObjects[0].data, Point(decodedObjects[0].location[3].x + 15, decodedObjects[0].location[3].y + 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,0), 2);
	}
}

int main()
{

	process_video();


}
