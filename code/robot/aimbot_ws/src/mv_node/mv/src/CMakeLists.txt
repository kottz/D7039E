

target_sources(${PROJECT_NAME} PUBLIC
"cv_main.cpp"
"config.cpp"
"qr_detector.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC
	"src"
)
