#include "carla/opendrive/OpenDriveParser.h"
#include "static_layers/map_data.h"

#include "renderarea.h"

#include <QtWidgets/QApplication>

#include <vector>
#include <chrono>
#include <iostream>

using namespace carla;

int main(int argc, char** argv)
{
  std::cout << argc << "," << argv[0] << ";";
  if (argc < 2)
  {
    std::cout << "Missing map\n";
    return -1;
  }
  std::string map_path(argv[1]);
  auto map_data = odv::load_map(map_path);
  // auto odr = opendrive::OpenDriveParser::LoadFile(argv[1]);
  QApplication app(argc, argv);
  RenderArea renderArea(map_data);
  renderArea.show();
  return app.exec();
}
