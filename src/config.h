#include "toml.hpp"
#include <loguru.hpp>
#include <string>
#include <any>

using namespace std;
class Config
{
public:

	Config();
	
	~Config();
	static Config GetConfigFromFile(const char* config_file);
	std::string GetVideoSource();
	bool DrawGUI();

private:
	Config(string video_source, bool draw_gui);
	string m_video_source;
	bool m_draw_gui;

};
