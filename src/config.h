
#include "toml.hpp"
#include <loguru.hpp>
#include <string>
#include <any>
#include "util.h"
#include "opencv2/opencv.hpp"

using namespace std;
class Config
{
public:

	Config();
	
	~Config();
	static Config GetConfigFromFile(const char* config_file);
	std::string GetVideoSource();
	bool DrawGUI();
	vector<mask> GetMask();
	cv::Rect GetGoalRect();

private:
	Config(string video_source, bool draw_gui, vector<vector<vector<int>>> masks, vector<int> goal_rect);
	string m_video_source;
	bool m_draw_gui;
	vector<vector<vector<int>>> m_masks;
	vector<int> m_goal_rect;

};
