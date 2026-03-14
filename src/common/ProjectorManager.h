#pragma once

#include <QObject>
#include <QStringList>
#include <memory>
#include <mutex>
#include <thread>

class JMTIController;

class ProjectorManager : public QObject {
    Q_OBJECT
public:
    static constexpr int PRO1_IDX = 0;
    static constexpr int PRO2_IDX = 1;
    static constexpr int TIMEOUT_MS = 10000;

    explicit ProjectorManager(QObject* parent = nullptr);
    ~ProjectorManager() noexcept override;

    // Synchronous queries (UI-thread safe, non-blocking)
    bool isConnected(int projIdx) const;
    int connectedCount() const;

public slots:
    void scanDevices();
    void connectDevice(int projIdx);
    void disconnectDevice(int projIdx);
    void connectAll();
    void disconnectAll();
    void showTestPattern(int projIdx, int patternCode, int colorCode);
    void stopTestPattern(int projIdx);
    void startPatternSequence(int projIdx, bool runOnce);
    void stopPatternSequence(int projIdx);
    void triggerSequential(bool runOnce, int delayMs);
    void triggerAll(bool runOnce);

signals:
    void deviceListUpdated(QStringList devices);
    void connectionStateChanged(int projIdx, bool connected);
    void statusMessage(QString msg);
    void errorOccurred(int projIdx, QString err);
    void allConnected();
    void allDisconnected();
    void operationFinished();

private:
    std::unique_ptr<JMTIController> controller_;
    mutable std::mutex mutex_;
};
