#include <zbar.h>
#include "opencv2/opencv.hpp"
#include <opencv2/objdetect.hpp>
using namespace zbar;
using namespace std;
using namespace cv;

typedef struct {
	string type;
	string data;
	vector<Point> location;
} decodedObject;

class QRDetector
{
public:

	QRDetector();
	
	~QRDetector();

    bool Detect(const Mat &img, vector<decodedObject>* decoded_objects);

private:
    ImageScanner m_scanner;
    vector<decodedObject> m_decodedObjects;
};