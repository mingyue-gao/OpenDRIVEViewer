#include "carla/opendrive/OpenDriveParser.h"

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
  auto odr = opendrive::OpenDriveParser::LoadFile(argv[1]);
  QApplication app(argc, argv);
  RenderArea renderArea(*odr);
  renderArea.show();
  return app.exec();
}
