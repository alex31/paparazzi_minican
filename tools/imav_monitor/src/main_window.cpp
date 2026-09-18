#include "main_window.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QStatusBar>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

QString percentage(const std::optional<float>& value) {
  if (!value.has_value()) {
    return "—";
  }
  return QString::number(std::lround(*value * 100.0F)) + "%";
}

} // namespace

MainWindow::MainWindow(MonitorOptions options, QWidget* parent)
  : QMainWindow(parent), options_(std::move(options)) {
  buildUi();

  receiver_.setCallback(
    [this](const ImavKeyValue& value) { handleKeyValue(value); });

  pollTimer_ = new QTimer(this);
  pollTimer_->setInterval(5);
  connect(pollTimer_, &QTimer::timeout, this,
          [this] { pollCan(); });
  pollTimer_->start();

  sampleTimer_ = new QTimer(this);
  sampleTimer_->setInterval(100);
  connect(sampleTimer_, &QTimer::timeout, this,
          [this] { sampleState(); });
  sampleTimer_->start();

  reconnectTimer_ = new QTimer(this);
  reconnectTimer_->setInterval(2000);
  connect(reconnectTimer_, &QTimer::timeout, this,
          [this] { connectCan(); });

  statusTimer_ = new QTimer(this);
  statusTimer_->setInterval(500);
  connect(statusTimer_, &QTimer::timeout, this,
          [this] { updateStatus(); });
  statusTimer_->start();

  QTimer::singleShot(0, this, [this] { connectCan(); });
}

MainWindow::~MainWindow() { receiver_.close(); }

void MainWindow::buildUi() {
  setWindowTitle("IMAV — Moniteur de détection");
  resize(1100, 680);

  auto* central = new QWidget(this);
  auto* root = new QVBoxLayout(central);
  root->setContentsMargins(24, 20, 24, 18);
  root->setSpacing(16);

  auto* header = new QHBoxLayout;
  auto* title = new QLabel("Détection de balise IMAV", central);
  QFont titleFont = title->font();
  titleFont.setPointSize(20);
  titleFont.setBold(true);
  title->setFont(titleFont);
  header->addWidget(title);
  header->addStretch();

  connectionLabel_ = new QLabel(central);
  connectionLabel_->setStyleSheet(
    "QLabel { color: #d1d5db; background: #374151; border-radius: 11px; "
    "padding: 5px 11px; }");
  header->addWidget(connectionLabel_);
  root->addLayout(header);

  auto* cards = new QHBoxLayout;
  cards->setSpacing(14);
  auto* audioContainer = new QFrame(central);
  auto* lightContainer = new QFrame(central);
  audioCard_ = createScoreCard("DÉTECTION SONORE", "#4ade80",
                               audioContainer);
  lightCard_ = createScoreCard("DÉTECTION LUMINEUSE", "#fbbf24",
                               lightContainer);
  cards->addWidget(audioContainer);
  cards->addWidget(lightContainer);
  root->addLayout(cards);

  auto* graphHeader = new QHBoxLayout;
  auto* graphTitle = new QLabel("Évolution temporelle", central);
  QFont graphFont = graphTitle->font();
  graphFont.setPointSize(13);
  graphFont.setBold(true);
  graphTitle->setFont(graphFont);
  graphHeader->addWidget(graphTitle);
  graphHeader->addStretch();
  auto* graphRange = new QLabel(
    QString("%1 dernières secondes").arg(options_.historySeconds, 0, 'f', 0),
    central);
  graphRange->setStyleSheet("color: #9ca3af;");
  graphHeader->addWidget(graphRange);
  root->addLayout(graphHeader);

  history_ = new HistoryWidget(options_.historySeconds, central);
  history_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  root->addWidget(history_, 1);

  statusLabel_ = new QLabel(central);
  statusLabel_->setStyleSheet("color: #9ca3af;");
  root->addWidget(statusLabel_);

  setCentralWidget(central);
  setStyleSheet(
    "QMainWindow, QWidget { background: #111827; color: #f3f4f6; }"
    "QFrame#scoreCard { background: #1f2937; border: 1px solid #374151; "
    "border-radius: 10px; }"
    "QProgressBar { background: #374151; border: 0; border-radius: 4px; "
    "height: 8px; text-align: center; }"
  );

  setScoreCard(audioCard_, std::nullopt, "En attente de snr");
  setScoreCard(lightCard_, std::nullopt, "En attente de lit");
  updateStatus();
}

MainWindow::ScoreCard MainWindow::createScoreCard(const QString& title,
                                                  const QString& color,
                                                  QWidget* parent) {
  parent->setObjectName("scoreCard");
  parent->setMinimumHeight(150);
  auto* layout = new QVBoxLayout(parent);
  layout->setContentsMargins(18, 15, 18, 15);
  layout->setSpacing(7);

  auto* titleLabel = new QLabel(title, parent);
  titleLabel->setStyleSheet("color: #9ca3af;");
  QFont labelFont = titleLabel->font();
  labelFont.setPointSize(9);
  labelFont.setBold(true);
  titleLabel->setFont(labelFont);
  layout->addWidget(titleLabel);

  auto* valueLabel = new QLabel("—", parent);
  QFont valueFont = valueLabel->font();
  valueFont.setPointSize(28);
  valueFont.setBold(true);
  valueLabel->setFont(valueFont);
  valueLabel->setStyleSheet(QString("color: %1;").arg(color));
  layout->addWidget(valueLabel);

  auto* detailLabel = new QLabel(parent);
  detailLabel->setStyleSheet("color: #d1d5db;");
  layout->addWidget(detailLabel);

  auto* bar = new QProgressBar(parent);
  bar->setRange(0, 1000);
  bar->setTextVisible(false);
  bar->setStyleSheet(QString(
    "QProgressBar { background: #374151; border: 0; border-radius: 4px; }"
    "QProgressBar::chunk { background: %1; border-radius: 4px; }").arg(color));
  layout->addWidget(bar);

  return {valueLabel, detailLabel, bar};
}

void MainWindow::setScoreCard(const ScoreCard& card,
                              const std::optional<float>& value,
                              const QString& detail) {
  card.value->setText(percentage(value));
  card.detail->setText(detail);
  card.bar->setValue(value.has_value()
    ? static_cast<int>(std::lround(
        std::clamp(*value, 0.0F, 1.0F) * 1000.0F))
    : 0);
}

void MainWindow::connectCan() {
  if (receiver_.isOpen()) {
    reconnectTimer_->stop();
    return;
  }

  std::string error;
  if (receiver_.open(options_.interfaceName.toStdString(),
                     options_.sourceNodeId, error)) {
    connectionError_.clear();
    previousAcceptedCount_ = 0;
    previousRateTime_ = DetectionState::Clock::now();
    reconnectTimer_->stop();
  } else {
    connectionError_ = QString::fromStdString(error);
    if (!reconnectTimer_->isActive()) {
      reconnectTimer_->start();
    }
  }
  updateStatus();
}

void MainWindow::pollCan() {
  if (!receiver_.isOpen()) {
    return;
  }
  if (!receiver_.pump()) {
    connectionError_ = QString::fromStdString(receiver_.lastError());
    receiver_.close();
    if (!reconnectTimer_->isActive()) {
      reconnectTimer_->start();
    }
    updateStatus();
  }
}

void MainWindow::sampleState() {
  const auto now = DetectionState::Clock::now();
  const DetectionSnapshot snapshot = state_.snapshot(now);
  const double time = elapsedSeconds(now);

  DetectionHistoryPoint point;
  point.timeSeconds = time;
  if (snapshot.audioScore.has_value()) {
    point.audio = *snapshot.audioScore;
    point.audioValid = true;
  }
  if (snapshot.lightScore.has_value()) {
    point.light = *snapshot.lightScore;
    point.lightValid = true;
  }
  history_->append(point);
  history_->setNow(time);

  QString audioDetail = "Donnée périmée";
  if (snapshot.audioScore.has_value()) {
    audioDetail = snapshot.snrDb.has_value()
      ? QString("SNR relatif %1 dB").arg(*snapshot.snrDb, 0, 'f', 1)
      : "SNR en attente";
  }
  setScoreCard(audioCard_, snapshot.audioScore, audioDetail);
  setScoreCard(lightCard_, snapshot.lightScore,
               snapshot.lightScore.has_value()
                 ? "Score du motif lumineux reconnu"
                 : "Donnée périmée");
}

void MainWindow::updateStatus() {
  const auto now = DetectionState::Clock::now();
  const double rateElapsed = std::chrono::duration<double>(
    now - previousRateTime_).count();
  if (receiver_.isOpen() && rateElapsed > 0.0) {
    const uint64_t accepted = receiver_.acceptedValueCount();
    receiveRateHz_ = static_cast<double>(accepted - previousAcceptedCount_) /
      rateElapsed;
    previousAcceptedCount_ = accepted;
    previousRateTime_ = now;
  }

  if (receiver_.isOpen()) {
    connectionLabel_->setText(
      QString("● %1 · nœud %2")
        .arg(options_.interfaceName)
        .arg(options_.sourceNodeId));
    connectionLabel_->setStyleSheet(
      "QLabel { color: #bbf7d0; background: #14532d; border-radius: 11px; "
      "padding: 5px 11px; }");
  } else {
    connectionLabel_->setText(
      QString("● %1 déconnecté").arg(options_.interfaceName));
    connectionLabel_->setStyleSheet(
      "QLabel { color: #fecaca; background: #7f1d1d; border-radius: 11px; "
      "padding: 5px 11px; }");
  }

  QString age = "aucune mesure nominale reçue";
  if (const auto last = state_.lastNominalUpdate(); last.has_value()) {
    age = QString("dernière mesure il y a %1 ms")
      .arg(std::max<qint64>(0, std::chrono::duration_cast<
        std::chrono::milliseconds>(now - *last).count()));
  }
  QString status = QString("%1 · %2 · %3 valeurs/s · %4 valeurs décodées")
    .arg(receiver_.isOpen() ? "CAN actif" : "CAN indisponible")
    .arg(age)
    .arg(receiveRateHz_, 0, 'f', 1)
    .arg(receiver_.acceptedValueCount());
  if (!connectionError_.isEmpty()) {
    status += " · " + connectionError_;
  }
  statusLabel_->setText(status);
}

void MainWindow::handleKeyValue(const ImavKeyValue& value) {
  state_.update(value.key, value.value, DetectionState::Clock::now());
}

double MainWindow::elapsedSeconds(DetectionState::TimePoint time) const {
  return std::chrono::duration<double>(time - startTime_).count();
}
