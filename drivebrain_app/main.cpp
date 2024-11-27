#include "DriveBrainApp.hpp"
#include <spdlog/spdlog.h>

int main(int argc, char* argv[]) {
    try {
        DriveBrainApp app(argc, argv);
        
        if (!app.initialize()) {
            spdlog::error("Failed to initialize DriveBrain application");
            return 1;
        }
        
        app.run();
        
        return 0;
    } catch (const std::exception& e) {
        spdlog::error("Fatal error: {}", e.what());
        return 1;
    }
}