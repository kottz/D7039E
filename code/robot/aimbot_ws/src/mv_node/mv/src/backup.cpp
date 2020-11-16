//ROS deps
#define NVIDIA

#if defined(NVIDIA)
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include "sensor_msgs/JointState.h"
#include <sstream>
#endif
#include "cv_main.h"
#include <chrono>
#include "readerwriterqueue.h"
#include "atomicops.h"
using namespace std::chrono; 

using namespace std;
using namespace cv;
using namespace zbar;
using namespace moodycamel;

#if defined(NVIDIA)
std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True";
}
#endif

Mat create_color_mask(Mat &img, vector<mask> &mask_vec) {
	int sizeX = img.cols;
	int sizeY = img.rows;

	int vertical_size = 70;
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
		cout << "data: " << decodedObjects[0].data << endl;
		Point center = QRDetector::GetCenter(decodedObjects[0]);
		if(center.y < sizeY/2) {
			track_point = center;
		}
		qr_tracked = true;
	}
	return track_point;
}

struct mv_output {
	std::string qr_data;
	float qr_x;
	float qr_y;
	int from_direction; //0 north, 1 east, 2 south, 3 west
};

bool follow_line_until_qr(BlockingReaderWriterQueue<Mat> &frame_q, BlockingReaderWriterQueue<int> &output_q, BlockingReaderWriterQueue<mv_output> &output_qr_q, vector<mask> &mask_vec, QRDetector &qr_detector, Rect goal_rect, bool draw_gui) {

	while (true) {
		auto start = high_resolution_clock::now();
		Mat frame;
		frame_q.wait_dequeue(frame);
		if (frame.empty()) {
				LOG_F(INFO, "Empty frame received. Stopping image processing.");
				break;
		}

		//resize(frame1, frame, Size(frame1.cols * 0.5, frame1.rows * 0.5), 0, 0, INTER_LINEAR);
		//cout << "width: " << frame.cols << endl;
		//cout << "height: " << frame.rows << endl;
		vector<decodedObject> decodedObjects;
		bool qr_tracked;
		Point track_point = trace_line_and_qr(frame, mask_vec, qr_detector, decodedObjects, qr_tracked);
		//Break when QR is in correct position
		
		//TODO: This has temporarily been removed since we just want to
		//send forever.
		/*
		if(qr_tracked && goal_rect.contains(track_point)) {
			LOG_F(INFO, "QR is in correct position. Stopping Robot.");
			return true;
		}
		*/
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
		int a = angle(track_point, Point(frame.cols/2, frame.rows));
		cout << "angle: " << a << endl;

		//TODO fixa qr code parsing json -> (x,y)
		if(qr_tracked) {
			decodedObject obj = decodedObjects[0];
			Point center = QRDetector::GetCenter(obj);
			mv_output out;
			out.qr_data = obj.data;
			out.qr_x = (float) center.x / frame.cols;
			out.qr_y = (float) center.y / frame.rows;
			out.from_direction = from_direction(obj);
			output_qr_q.try_enqueue(out);
			cout << "qr data: " << out.qr_data << endl;
			cout << "qr x: " << out.qr_x << endl;
			cout << "qr y: " << out.qr_y << endl;
			cout << "qr from dir: " << out.from_direction << endl;
		}
		output_q.try_enqueue(a);
		
		//video.write(frame);
		auto stop = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(stop - start);
		cout << "fps: " << 1000000/duration.count() << endl;
	}
	return false;
}

int process_video(ros::Publisher mv_pub, ros::Publisher qr_pub) {
	Config config = Config::GetConfigFromFile("config.toml");
	bool draw_gui = config.DrawGUI();
	vector<mask> mask_vec = config.GetMask();
	
	string video_name = config.GetVideoSource();
	
	VideoCapture cap = VideoCapture();

	if(video_name == "camera") {
#if defined(NVIDIA)
		int capture_width = 640 ;
		int capture_height = 480 ;
		int display_width = 360 ;
		int display_height = 240 ;
		int framerate = 90 ;
		int flip_method = 2 ;

		std::string pipeline = gstreamer_pipeline(capture_width,
		capture_height,
		display_width,
		display_height,
		framerate,
		flip_method);
		//pipeline = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12, framerate=(fraction)120/1 ! nvvidconv flip-method=0 ! appsink max-buffers=1 drop=True";
		std::cout << "Using pipeline: \n\t" << pipeline << "\n";
		
		//VideoCapture cap = VideoCapture();
		cap.open(pipeline, cv::CAP_GSTREAMER);
#else
		cout << "NU TAR DU FEL" << endl;
		cap.open(0);
		cap.set(CAP_PROP_FRAME_WIDTH,320);
		cap.set(CAP_PROP_FRAME_HEIGHT,240);
		cap.set(CAP_PROP_FPS, 90);
#endif
	} else {
		cap.open(video_name);
	}
	
	if (!cap.isOpened()) {
		LOG_F(ERROR, "Could not open video source");
		return -1;
	}
	BlockingReaderWriterQueue<Mat> frame_q(2);
	BlockingReaderWriterQueue<mv_output> output_qr_q(1);
	BlockingReaderWriterQueue<int> output_q(10);
	
	//create VideoCapture thread
	std::thread writer([&]() {
		while(true) {
			Mat img;
			cap >> img;
			if (img.empty()) {
				frame_q.enqueue(img); // enqueue this so that reader gets the empty frame
				LOG_F(INFO, "Empty frame captured. Stopping capture thread.");
				break;
			}
			//TODO: When processing a video this will skip frames.
			//Discard frame if queue is full
			bool a = frame_q.try_enqueue(img);
		}
	});
	//create image processing thread	
	std::thread image_processing([&]() {
		QRDetector qr_detector = QRDetector();
		Rect goal_rect = config.GetGoalRect();
		follow_line_until_qr(frame_q, output_q, output_qr_q, mask_vec, qr_detector, goal_rect, draw_gui);
	});

//Only send to ROS if we are on the Nvidia	
#if defined(NVIDIA)


	//Send ROS msg with angle
	while(true) {
		int angle;
		output_q.wait_dequeue(angle);
		std_msgs::Int32 ros_angle_msg;
		ros_angle_msg.data = angle;
		mv_pub.publish(ros_angle_msg);

		mv_output qr_msg;
		if(output_qr_q.try_dequeue(qr_msg)) {
			sensor_msgs::JointState qr;
			qr.name.push_back(qr_msg.qr_data);
			qr.position.push_back(qr_msg.qr_x);
			qr.position.push_back(qr_msg.qr_y);
			qr.effort.push_back(qr_msg.from_direction);
			qr_pub.publish(qr);
		}
	}
#endif	

	writer.join();
	image_processing.join();

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

int from_direction(decodedObject obj) {
	//find middle of each line, return the one which has the highest y pos (furthest down in real life)
	int y_max = 0;
	int dir = -1; //0 north, 1 west, 2 south, 3 east
	for(int i = 0; i < 4; i++) {
		int p = (obj.location[i].y + obj.location[(i+1)%4].y)/2;
		//cout << "y1 " << obj.location[i].y << " y2 " << obj.location[(i+1)%4].y << endl;
		//cout << "p " << p << "i " << i << endl;
		if(p > y_max) {
			y_max = p;
			dir = (i+1)%4; //zbar uses will set bottom left corner to index one. but we want to have north as 0
		}
	}
	//cout << "new y1 " << obj.location[0] << " y2 " << obj.location[1] << endl;
	//cout << "new y1 " << obj.location[1] << " y2 " << obj.location[2] << endl;
	//cout << "new y1 " << obj.location[2] << " y2 " << obj.location[3] << endl;
	//cout << "new y1 " << obj.location[3] << " y2 " << obj.location[0] << endl;
	return dir;
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
		int dir = from_direction(decodedObjects[0]);
		cout << dir << endl;
		putText(img, decodedObjects[0].data + " dir " + std::to_string(dir), Point(decodedObjects[0].location[3].x + 15, decodedObjects[0].location[3].y + 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0,0,0), 2);
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "mv");
	ros::NodeHandle n;
	ros::Publisher mv_pub = n.advertise<std_msgs::Int32>("mv", 1);
	ros::Publisher qr_pub = n.advertise<sensor_msgs::JointState>("mv_qr", 1);
	ros::Rate loop_rate(11); // Rate = 11 originally
	process_video(mv_pub, qr_pub);

}
