#include "cv_main.h"
#include <chrono>
using namespace std::chrono; 

using namespace std;
using namespace cv;
using namespace zbar;


int line_trace() {
	Config config = Config::GetConfigFromFile("config.toml");
	bool draw_gui = config.DrawGUI();
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
	Scalar mask0_low = Scalar(0, 50, 50);
	Scalar mask0_high = Scalar(10, 255, 255);

	Scalar mask1_low = Scalar(170, 50, 50);
	Scalar mask1_high = Scalar(180, 255, 255);
	
	QRDetector qr_detector = QRDetector();

	
	while (true) {
		auto start = high_resolution_clock::now();
		Mat frame1;
		cap >> frame1;
		Mat frame;
		
		if (frame1.empty())
			break;
		resize(frame1, frame, Size(frame1.cols * 0.5, frame1.rows * 0.5), 0, 0, INTER_LINEAR);
		//horizontal slice extraction	
		int sizeX = frame.cols;
		int sizeY = frame.rows;

		int camX = sizeX / 2;
		int camY = sizeY - 0;

		int vertical_size = 10;

		Rect myROI(0, (sizeY-vertical_size)/2, sizeX, vertical_size);
		Mat cropped = frame(myROI);
		Mat hsv;
		cvtColor(cropped, hsv, COLOR_BGR2HSV);

		Mat mask0;
		inRange(hsv, mask0_low, mask0_high, mask0);
		Mat mask1;
		inRange(hsv, mask1_low, mask1_high, mask1);
		Mat mask;
		bitwise_or(mask0, mask1, mask);
		
		int cX;
		Moments m = moments(mask);
		if(m.m00 == 0)
			cX = 0;
		else
			cX = m.m10 / m.m00;

		//cout << cX << endl;
		vec2f track_point = vec2f(cX, sizeY/2);
		vec2f cam = vec2f(camX, camY);
		



		//qr
		vector<decodedObject> decodedObjects;
		bool detected = qr_detector.Detect(frame, &decodedObjects);
		
		if(draw_gui) {
			drawInfo(frame, vecToPoint(cam), vecToPoint(track_point), decodedObjects);
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

void drawInfo(Mat &img, Point cam, Point track, const vector<decodedObject> &decodedObjects) {

	line(img, track, cam, Scalar(255,0,0),2);

	int radius = 80;
	vec2f camv = vec2f((float)cam.x, (float)cam.y);
	vec2f trackv = vec2f((float)track.x, (float)track.y);
	vec2f u = trackv - camv;
	u = u.normalize() * radius;
	float angle = u.argument();
	//cout << angle << endl;
	
	line(img, vecToPoint(camv), vecToPoint(camv+u), Scalar(0,0,0), 5);
	ellipse(img, vecToPoint(camv), Size(u.length(), u.length()), 0, 0, -angle, Scalar(0,0,0), 5);
	vec2f textOffset = vec2f(radius+10,-10);
	putText(img, to_string((int)angle), vecToPoint(camv+textOffset), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);

	
	if(decodedObjects.size() > 0) {
	//cout << "detected" << endl;
	//cout << decodedObjects[0].data << endl;
	//draw lines around qr code
	line(img, decodedObjects[0].location[0], decodedObjects[0].location[1], Scalar(255,0,0),3);
	line(img, decodedObjects[0].location[1], decodedObjects[0].location[2], Scalar(255,0,0),3);
	line(img, decodedObjects[0].location[2], decodedObjects[0].location[3], Scalar(255,0,0),3);
	line(img, decodedObjects[0].location[3], decodedObjects[0].location[0], Scalar(255,0,0),3);
	putText(img, decodedObjects[0].data, Point(decodedObjects[0].location[3].x + 15, decodedObjects[0].location[3].y + 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,0), 2);
	}
}

int main()
{
	line_trace();
}
