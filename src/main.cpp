#include <Stonefish/core/GraphicalSimulationApp.h>
#include "ocean_sim.hpp"
#include <iostream>
#include <filesystem>

int main(int argc, char **argv)
{
    //Using default settings
    sf::RenderSettings s;
    s.windowW = 1820;
    s.windowH = 1000;
    sf::HelperSettings h;

    try {
        std::filesystem::path cwd = std::filesystem::current_path();
        std::cout << "Current path is " << cwd << std::endl;
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    MySimulationManager manager(500.0);
    sf::GraphicalSimulationApp app("Simple simulator", "path_to_data", s, h, &manager);
    app.Run();

    return 0;
}
