

target_sources(${PROJECT_NAME} PUBLIC
"src/cv_main.cpp"
"src/config.cpp"
"src/qr_detector.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC
	"src"
)