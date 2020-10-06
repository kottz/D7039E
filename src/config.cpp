
#include "config.h"

Config::Config()
{

}

Config::Config(string video_source, bool draw_gui, vector<vector<vector<int>>> masks, vector<int> goal_rect)
{
	m_video_source = video_source;
	m_draw_gui = draw_gui;
	m_masks = masks;
	m_goal_rect = goal_rect;
}

Config::~Config()
{

}



std::string Config::GetVideoSource() {
    return m_video_source;
}

bool Config::DrawGUI() {
	return m_draw_gui;
}

vector<mask> Config::GetMask() {
	vector<mask> v;
	
	for(auto x = 0; x < m_masks.size(); x++ ) {
		v.push_back({.low = cv::Scalar(m_masks[x][0][0], m_masks[x][0][1], m_masks[x][0][2]),
		 				.high = cv::Scalar(m_masks[x][1][0], m_masks[x][1][1], m_masks[x][1][2])});
	}
	return v;
}

cv::Rect Config::GetGoalRect() {
	return cv::Rect(m_goal_rect[0],
					m_goal_rect[1],
					m_goal_rect[2],
					m_goal_rect[3]
	);
}

Config Config::GetConfigFromFile(const char* config_file) {
	std::unordered_map<std::string, std::any> params;

	std::string cf = config_file;
	auto data					= toml::parse(cf);

	const auto general					= toml::find(data, "general");
	std::string video_source			= toml::find<std::string>(general, "video_source");
	bool gui							= toml::find<bool>(general, "gui");
	vector<vector<vector<int>>>	masks	= toml::find<vector<vector<vector<int>>>>(general, "masks");
	
	for(int i = 0; i < masks.size(); i++) {
		if(masks[i].size() != 2) {
			LOG_F(ERROR, "mask config size incorrect. only high and low.");
			exit(2);
		}
		
		for(int y = 0; y < masks[i].size(); y++) {
			if(masks[i][y].size() != 3) {
				LOG_F(ERROR, "mask config size incorrect. Each scalar should have 3 elements");
				exit(2);
			}
		}
	}

	vector<int>	goal_rect				= toml::find<vector<int>>(general, "goal_rect");
	if(goal_rect.size() != 4) {
		LOG_F(ERROR, "Rect settings are invalid. Should contain 4 points.");
		exit(2);
	}
	//params["NO_LENS_DISTORTION"]	= toml::find<bool>(general, "no_lens_distortion");

	return Config(video_source, gui, masks, goal_rect);
}