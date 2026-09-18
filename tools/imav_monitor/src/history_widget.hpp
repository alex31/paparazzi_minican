#pragma once

#include <QWidget>

#include <deque>

struct DetectionHistoryPoint {
  double timeSeconds = 0.0;
  float audio = 0.0F;
  float light = 0.0F;
  bool audioValid = false;
  bool lightValid = false;
};

class HistoryWidget final : public QWidget {
public:
  explicit HistoryWidget(double historySeconds, QWidget* parent = nullptr);

  void append(const DetectionHistoryPoint& point);
  void setNow(double nowSeconds);
  QSize minimumSizeHint() const override;

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  double historySeconds_;
  double nowSeconds_ = 0.0;
  std::deque<DetectionHistoryPoint> points_;
};
