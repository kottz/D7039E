#include "qr_detector.h"

QRDetector::QRDetector()
{
    m_scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
	m_scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
}

QRDetector::~QRDetector() {}

bool QRDetector::Detect(const Mat &img, vector<decodedObject>* decoded_objects) {
    Mat gray_img;
    cvtColor(img, gray_img, COLOR_BGR2GRAY);
    Image image(img.cols, img.rows, "Y800", (uchar *)gray_img.data, img.cols * img.rows);
    int n = m_scanner.scan(image);
    if (n <= 0)
        return false;
    
    for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
        decodedObject obj;
        obj.type = symbol->get_type_name();
        obj.data = symbol->get_data();
        // Print type and data
        /*
        cout << "Type : " << obj.type << endl;
        cout << "locX : " << symbol->get_location_x(0) << endl;
        cout << "locY : " << symbol->get_location_y(0) << endl;
        cout << "size : " << symbol->get_location_size() << endl;
        cout << "Data : " << obj.data << endl << endl;
        */
        // Obtain location
	    for(int i = 0; i< symbol->get_location_size(); i++) {
	        obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
	    }

        decoded_objects->push_back(obj);
    }
    return true;
}
