
#include "config.h"

Config::Config()
{

}

Config::Config(string video_source, bool draw_gui)
{
	m_video_source = video_source;
	m_draw_gui = draw_gui;
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


Config Config::GetConfigFromFile(const char* config_file) {
	std::unordered_map<std::string, std::any> params;

	std::string cf = config_file;
	auto data					= toml::parse(cf);

	const auto general				= toml::find(data, "general");
	std::string video_source		= toml::find<std::string>(general, "video_source");
	bool gui						= toml::find<bool>(general, "gui");
	//params["NO_LENS_DISTORTION"]	= toml::find<bool>(general, "no_lens_distortion");

	return Config(video_source, gui);
}