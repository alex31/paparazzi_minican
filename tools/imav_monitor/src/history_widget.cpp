#include "history_widget.hpp"

#include <QFontMetrics>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>

#include <algorithm>
#include <cmath>
#include <functional>

namespace {

constexpr QColor kAudioColor(74, 222, 128);
constexpr QColor kLightColor(251, 191, 36);
constexpr QColor kGridColor(55, 65, 81);
constexpr QColor kTextColor(156, 163, 175);

} // namespace

HistoryWidget::HistoryWidget(double historySeconds, QWidget* parent)
  : QWidget(parent), historySeconds_(historySeconds) {
  setAutoFillBackground(false);
}

void HistoryWidget::append(const DetectionHistoryPoint& point) {
  points_.push_back(point);
  const double oldest = point.timeSeconds - historySeconds_;
  while (!points_.empty() && points_.front().timeSeconds < oldest) {
    points_.pop_front();
  }
  update();
}

void HistoryWidget::setNow(double nowSeconds) {
  nowSeconds_ = nowSeconds;
  const double oldest = nowSeconds_ - historySeconds_;
  while (!points_.empty() && points_.front().timeSeconds < oldest) {
    points_.pop_front();
  }
  update();
}

QSize HistoryWidget::minimumSizeHint() const { return {620, 300}; }

void HistoryWidget::paintEvent(QPaintEvent*) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), QColor(17, 24, 39));

  const int leftMargin = 54;
  const int rightMargin = 18;
  const int topMargin = 42;
  const int bottomMargin = 34;
  const QRectF plot(leftMargin, topMargin,
                    std::max(1, width() - leftMargin - rightMargin),
                    std::max(1, height() - topMargin - bottomMargin));

  painter.setPen(QPen(kGridColor, 1));
  painter.setFont(QFont(painter.font().family(), 9));
  for (int percent = 0; percent <= 100; percent += 25) {
    const qreal y = plot.bottom() - plot.height() * percent / 100.0;
    painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    painter.setPen(kTextColor);
    painter.drawText(QRectF(0, y - 9, leftMargin - 8, 18),
                     Qt::AlignRight | Qt::AlignVCenter,
                     QString::number(percent) + "%");
    painter.setPen(QPen(kGridColor, 1));
  }

  constexpr int xGridSeconds = 5;
  const int gridCount = static_cast<int>(historySeconds_) / xGridSeconds;
  for (int index = 0; index <= gridCount; ++index) {
    const double age = static_cast<double>(index * xGridSeconds);
    const qreal x = plot.right() - plot.width() * age / historySeconds_;
    painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
    painter.setPen(kTextColor);
    painter.drawText(QRectF(x - 30, plot.bottom() + 7, 60, 18),
                     Qt::AlignHCenter | Qt::AlignTop,
                     age == 0.0 ? "0 s" : QString("-%1 s").arg(age));
    painter.setPen(QPen(kGridColor, 1));
  }

  auto drawLegendItem = [&painter](int x, const QColor& color,
                                    const QString& text) {
    painter.setPen(QPen(color, 3));
    painter.drawLine(x, 20, x + 22, 20);
    painter.setPen(QColor(229, 231, 235));
    painter.drawText(x + 29, 26, text);
  };
  const QString audioLegend = "Présence sonore";
  drawLegendItem(leftMargin, kAudioColor, audioLegend);
  drawLegendItem(leftMargin + 29 +
                   painter.fontMetrics().horizontalAdvance(audioLegend) + 24,
                 kLightColor, "Score lumineux");

  const double windowStart = nowSeconds_ - historySeconds_;
  auto mapPoint = [&](double time, float value) {
    const qreal x = plot.left() + plot.width() *
      (time - windowStart) / historySeconds_;
    const qreal y = plot.bottom() - plot.height() *
      std::clamp(static_cast<double>(value), 0.0, 1.0);
    return QPointF(x, y);
  };

  auto drawCurve = [&](const QColor& color,
                       const std::function<bool(
                         const DetectionHistoryPoint&)>& valid,
                       const std::function<float(
                         const DetectionHistoryPoint&)>& value) {
    QPainterPath path;
    bool pathActive = false;
    for (const auto& point : points_) {
      if (point.timeSeconds < windowStart || !valid(point)) {
        pathActive = false;
        continue;
      }
      const QPointF mapped = mapPoint(point.timeSeconds, value(point));
      if (!pathActive) {
        path.moveTo(mapped);
        pathActive = true;
      } else {
        path.lineTo(mapped);
      }
    }
    painter.setPen(QPen(color, 2.4, Qt::SolidLine, Qt::RoundCap,
                        Qt::RoundJoin));
    painter.drawPath(path);
  };

  painter.save();
  painter.setClipRect(plot.adjusted(-2, -2, 2, 2));
  drawCurve(kAudioColor,
            [](const DetectionHistoryPoint& p) { return p.audioValid; },
            [](const DetectionHistoryPoint& p) { return p.audio; });
  drawCurve(kLightColor,
            [](const DetectionHistoryPoint& p) { return p.lightValid; },
            [](const DetectionHistoryPoint& p) { return p.light; });
  painter.restore();

  painter.setPen(QPen(QColor(75, 85, 99), 1));
  painter.drawRect(plot);

  if (points_.empty()) {
    painter.setPen(kTextColor);
    painter.setFont(QFont(painter.font().family(), 13));
    painter.drawText(plot, Qt::AlignCenter,
                     "En attente des mesures IMAV sur CAN");
  }
}
