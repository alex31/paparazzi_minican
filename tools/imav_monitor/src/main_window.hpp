#pragma once

#include "detection_state.hpp"
#include "history_widget.hpp"
#include "uavcan_receiver.hpp"

#include <QMainWindow>

#include <chrono>
#include <cstdint>

class QLabel;
class QProgressBar;
class QTimer;

struct MonitorOptions {
  QString interfaceName = "can0";
  uint8_t sourceNodeId = 10;
  double historySeconds = 30.0;
};

class MainWindow final : public QMainWindow {
public:
  explicit MainWindow(MonitorOptions options, QWidget* parent = nullptr);
  ~MainWindow() override;

private:
  struct ScoreCard {
    QLabel* value = nullptr;
    QLabel* detail = nullptr;
    QProgressBar* bar = nullptr;
  };

  void buildUi();
  ScoreCard createScoreCard(const QString& title,
                            const QString& color,
                            QWidget* parent);
  void setScoreCard(const ScoreCard& card,
                    const std::optional<float>& value,
                    const QString& detail);
  void connectCan();
  void pollCan();
  void sampleState();
  void updateStatus();
  void handleKeyValue(const ImavKeyValue& value);
  double elapsedSeconds(DetectionState::TimePoint time) const;

  MonitorOptions options_;
  DetectionState state_;
  UavcanReceiver receiver_;
  DetectionState::TimePoint startTime_ = DetectionState::Clock::now();

  QTimer* pollTimer_ = nullptr;
  QTimer* sampleTimer_ = nullptr;
  QTimer* reconnectTimer_ = nullptr;
  QTimer* statusTimer_ = nullptr;
  QLabel* connectionLabel_ = nullptr;
  QLabel* statusLabel_ = nullptr;
  ScoreCard audioCard_;
  ScoreCard lightCard_;
  HistoryWidget* history_ = nullptr;

  uint64_t previousAcceptedCount_ = 0;
  DetectionState::TimePoint previousRateTime_ = DetectionState::Clock::now();
  double receiveRateHz_ = 0.0;
  QString connectionError_;
};
