#include "main_window.hpp"

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QPalette>
#include <QStyleFactory>

namespace {

void applyDarkPalette(QApplication& application) {
  application.setStyle(QStyleFactory::create("Fusion"));
  QPalette palette;
  palette.setColor(QPalette::Window, QColor(17, 24, 39));
  palette.setColor(QPalette::WindowText, QColor(243, 244, 246));
  palette.setColor(QPalette::Base, QColor(31, 41, 55));
  palette.setColor(QPalette::AlternateBase, QColor(55, 65, 81));
  palette.setColor(QPalette::Text, QColor(243, 244, 246));
  palette.setColor(QPalette::Button, QColor(31, 41, 55));
  palette.setColor(QPalette::ButtonText, QColor(243, 244, 246));
  palette.setColor(QPalette::Highlight, QColor(37, 99, 235));
  palette.setColor(QPalette::HighlightedText, Qt::white);
  application.setPalette(palette);
}

} // namespace

int main(int argc, char* argv[]) {
  QApplication application(argc, argv);
  QApplication::setApplicationName("imav_monitor");
  QApplication::setApplicationVersion("1.0");

  QCommandLineParser parser;
  parser.setApplicationDescription(
    "Moniteur Qt des détections IMAV reçues sur DroneCAN");
  parser.addHelpOption();
  parser.addVersionOption();
  const QCommandLineOption interfaceOption(
    {"i", "interface"}, "Interface SocketCAN (défaut: can0).", "nom",
    "can0");
  const QCommandLineOption sourceOption(
    {"n", "source-node"}, "Identifiant du nœud MicroCAN (défaut: 10).",
    "id", "10");
  const QCommandLineOption historyOption(
    {"t", "history"}, "Durée du graphe en secondes (défaut: 30).",
    "secondes", "30");
  parser.addOption(interfaceOption);
  parser.addOption(sourceOption);
  parser.addOption(historyOption);
  parser.process(application);

  bool nodeOk = false;
  const int node = parser.value(sourceOption).toInt(&nodeOk);
  bool historyOk = false;
  const double history = parser.value(historyOption).toDouble(&historyOk);
  if (!nodeOk || node < 1 || node > 127) {
    parser.showHelp(2);
  }
  if (!historyOk || history < 5.0 || history > 300.0) {
    parser.showHelp(2);
  }

  MonitorOptions options;
  options.interfaceName = parser.value(interfaceOption);
  options.sourceNodeId = static_cast<uint8_t>(node);
  options.historySeconds = history;

  applyDarkPalette(application);
  MainWindow window(options);
  window.show();
  return application.exec();
}
