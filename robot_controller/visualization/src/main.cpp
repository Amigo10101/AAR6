#include "AppState.h"
#include "MotionController.h"
#include "MotionPlannerModel.h"
#include "RobotManager.h"
#include "RobotNode.h"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickWindow>
#include <iostream>

int main(int argc, char *argv[]) {
  QGuiApplication app(argc, argv);

  // Print which graphics API Qt will use
  auto api = QQuickWindow::graphicsApi();
  std::cout << "Qt Graphics API: ";
  switch (api) {
  case QSGRendererInterface::OpenGL:
    std::cout << "OpenGL (GPU)";
    break;
  case QSGRendererInterface::Vulkan:
    std::cout << "Vulkan (GPU)";
    break;
  case QSGRendererInterface::Direct3D11:
    std::cout << "Direct3D 11 (GPU)";
    break;
  case QSGRendererInterface::Metal:
    std::cout << "Metal (GPU)";
    break;
  case QSGRendererInterface::Software:
    std::cout << "Software (CPU - slow!)";
    break;
  default:
    std::cout << "Unknown (" << static_cast<int>(api) << ")";
    break;
  }
  std::cout << std::endl;

  // Register QML types
  qmlRegisterType<RobotNode>("RobotVisualization", 1, 0, "RobotNode");
  qmlRegisterType<WaypointListModel>("RobotVisualization", 1, 0,
                                     "WaypointListModel");
  qmlRegisterType<AppState>("RobotVisualization", 1, 0, "AppState");
  qmlRegisterType<MotionController>("RobotVisualization", 1, 0,
                                    "MotionController");

  QQmlApplicationEngine engine;

  // Create RobotManager instance
  RobotManager robotManager;
  engine.rootContext()->setContextProperty("robotManager", &robotManager);

  // Pass command line args to QML
  QStringList args;
  for (int i = 0; i < argc; ++i)
    args << argv[i];
  engine.rootContext()->setContextProperty("appArgs", args);

  // Load main.qml from the embedded resource system
  const QUrl url("qrc:/Visualization/qml/main.qml");

  QObject::connect(
      &engine, &QQmlApplicationEngine::objectCreated, &app,
      [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl) {
          std::cerr << "Failed to load QML component from: "
                    << url.toString().toStdString() << std::endl;
          QCoreApplication::exit(-1);
        }
      },
      Qt::QueuedConnection);

  engine.load(url);

  return app.exec();
}
