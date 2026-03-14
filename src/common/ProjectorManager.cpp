#include "ProjectorManager.h"
#include "../../Project2/Project2/JMTIController.h"
#include <QDebug>
#include <QThread>
#include <thread>

ProjectorManager::ProjectorManager(QObject* parent)
    : QObject(parent)
    , controller_(std::make_unique<JMTIController>())
{
    qDebug() << "[ProjectorManager] Constructed, parent:" << parent;
}

ProjectorManager::~ProjectorManager() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    if (controller_) {
        controller_->DisconnectAllDevices();
    }
}

bool ProjectorManager::isConnected(int projIdx) const {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return false;
    return controller_->IsDeviceConnected(projIdx);
}

int ProjectorManager::connectedCount() const {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return 0;
    int count = 0;
    int total = controller_->GetDeviceCount();
    for (int i = 0; i < total; ++i) {
        if (controller_->IsDeviceConnected(i)) ++count;
    }
    return count;
}

void ProjectorManager::scanDevices() {
    qDebug() << "[ProjectorManager] scanDevices() called from thread:" << QThread::currentThread();
    emit statusMessage(QStringLiteral("正在扫描投影仪设备..."));

    // 在后台线程执行硬件操作，避免阻塞 UI
    std::thread([this]() {
        qDebug() << "[ProjectorManager] scanDevices worker thread started";
        bool ok = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ok = controller_->EnumerateDevices();
        }

        if (!ok) {
            qDebug() << "[ProjectorManager] EnumerateDevices returned false";
            emit statusMessage(QStringLiteral("未发现投影仪设备"));
            emit errorOccurred(-1, QStringLiteral("扫描失败：未检测到可用的 Cypress I2C 投影设备"));
            emit deviceListUpdated({});
        } else {
            std::lock_guard<std::mutex> lock(mutex_);
            auto devList = controller_->GetDeviceList();
            QStringList qList;
            for (const auto& d : devList) {
                qList.append(QString::fromStdString(d));
            }
            qDebug() << "[ProjectorManager] Found" << qList.size() << "devices";
            emit statusMessage(QStringLiteral("发现 %1 台投影仪").arg(qList.size()));
            emit deviceListUpdated(qList);
        }
        emit operationFinished();
    }).detach();
}

void ProjectorManager::connectDevice(int projIdx) {
    qDebug() << "[ProjectorManager] connectDevice(" << projIdx << ")";
    emit statusMessage(QStringLiteral("正在连接 Pro%1...").arg(projIdx + 1));

    std::thread([this, projIdx]() {
        qDebug() << "[ProjectorManager] connectDevice worker thread for Pro" << (projIdx + 1);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (controller_->GetDeviceCount() <= 0) {
                emit errorOccurred(projIdx, QStringLiteral("请先扫描设备，再执行连接"));
                emit connectionStateChanged(projIdx, false);
                emit operationFinished();
                return;
            }
        }

        bool ok = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ok = controller_->OpenTI(projIdx);
        }

        if (ok) {
            qDebug() << "[ProjectorManager] Pro" << (projIdx + 1) << "connected OK";
            emit statusMessage(QStringLiteral("Pro%1 连接成功").arg(projIdx + 1));
            emit connectionStateChanged(projIdx, true);
        } else {
            qDebug() << "[ProjectorManager] Pro" << (projIdx + 1) << "connect FAILED";
            emit errorOccurred(projIdx,
                QStringLiteral("Pro%1 连接失败").arg(projIdx + 1));
            emit connectionStateChanged(projIdx, false);
        }
        emit operationFinished();
    }).detach();
}

void ProjectorManager::disconnectDevice(int projIdx) {
    qDebug() << "[ProjectorManager] disconnectDevice(" << projIdx << ")";
    std::thread([this, projIdx]() {
        std::lock_guard<std::mutex> lock(mutex_);
        bool ok = controller_->DisconnectDevice(projIdx);
        if (ok) {
            emit statusMessage(QStringLiteral("Pro%1 已断开").arg(projIdx + 1));
        } else {
            emit errorOccurred(projIdx,
                QStringLiteral("Pro%1 断开失败").arg(projIdx + 1));
        }
        emit connectionStateChanged(projIdx, false);
        emit operationFinished();
    }).detach();
}

void ProjectorManager::connectAll() {
    qDebug() << "[ProjectorManager] connectAll()";
    emit statusMessage(QStringLiteral("正在连接全部投影仪..."));

    std::thread([this]() {
        qDebug() << "[ProjectorManager] connectAll worker thread started";
        std::lock_guard<std::mutex> lock(mutex_);
        int total = controller_->GetDeviceCount();
        if (total <= 0) {
            emit errorOccurred(-1, QStringLiteral("请先扫描设备，再执行全部连接"));
            emit operationFinished();
            return;
        }

        int connected = 0;
        for (int i = 0; i < total; ++i) {
            bool ok = controller_->OpenTI(i);
            if (ok) ++connected;
            qDebug() << "[ProjectorManager] OpenTI(" << i << ") =>" << ok;
            emit connectionStateChanged(i, ok);
            if (ok) {
                emit statusMessage(QStringLiteral("Pro%1 连接成功").arg(i + 1));
            } else {
                emit errorOccurred(i, QStringLiteral("Pro%1 连接失败").arg(i + 1));
            }
        }

        if (connected == total && total > 0) {
            emit allConnected();
        }
        emit operationFinished();
    }).detach();
}

void ProjectorManager::disconnectAll() {
    qDebug() << "[ProjectorManager] disconnectAll()";
    std::thread([this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        controller_->DisconnectAllDevices();
        int total = controller_->GetDeviceCount();
        for (int i = 0; i < total; ++i) {
            emit connectionStateChanged(i, false);
        }
        emit statusMessage(QStringLiteral("全部投影仪已断开"));
        emit allDisconnected();
        emit operationFinished();
    }).detach();
}

void ProjectorManager::showTestPattern(int projIdx, int patternCode, int colorCode) {
    std::thread([this, projIdx, patternCode, colorCode]() {
        std::lock_guard<std::mutex> lock(mutex_);
        auto pattern = static_cast<DlpcTestPattern>(patternCode);
        auto color = static_cast<DlpcColor>(colorCode);
        bool ok = controller_->StartTestPattern(projIdx, pattern, color);
        if (ok) {
            emit statusMessage(QStringLiteral("Pro%1 测试图案已显示").arg(projIdx + 1));
        } else {
            emit errorOccurred(projIdx,
                QStringLiteral("Pro%1 显示测试图案失败").arg(projIdx + 1));
        }
    }).detach();
}

void ProjectorManager::stopTestPattern(int projIdx) {
    std::thread([this, projIdx]() {
        std::lock_guard<std::mutex> lock(mutex_);
        bool ok = controller_->StopProjection(projIdx);
        if (ok) {
            emit statusMessage(QStringLiteral("Pro%1 测试图案已停止").arg(projIdx + 1));
        } else {
            emit errorOccurred(projIdx,
                QStringLiteral("Pro%1 停止测试图案失败").arg(projIdx + 1));
        }
    }).detach();
}

void ProjectorManager::startPatternSequence(int projIdx, bool runOnce) {
    std::thread([this, projIdx, runOnce]() {
        std::lock_guard<std::mutex> lock(mutex_);
        bool ok = controller_->slotTriggerITBoard(projIdx);
        if (!ok) {
            emit errorOccurred(projIdx,
                QStringLiteral("Pro%1 触发条纹投射失败").arg(projIdx + 1));
            return;
        }
        ok = controller_->RunPatternSequence(projIdx, runOnce);
        if (ok) {
            emit statusMessage(QStringLiteral("Pro%1 条纹投射已启动 (%2)")
                .arg(projIdx + 1)
                .arg(runOnce ? QStringLiteral("单次") : QStringLiteral("循环")));
        } else {
            emit errorOccurred(projIdx,
                QStringLiteral("Pro%1 运行图案序列失败").arg(projIdx + 1));
        }
    }).detach();
}

void ProjectorManager::stopPatternSequence(int projIdx) {
    std::thread([this, projIdx]() {
        std::lock_guard<std::mutex> lock(mutex_);
        bool ok = controller_->slotTriggerStop(projIdx);
        if (ok) {
            emit statusMessage(QStringLiteral("Pro%1 投射已停止").arg(projIdx + 1));
        } else {
            emit errorOccurred(projIdx,
                QStringLiteral("Pro%1 停止投射失败").arg(projIdx + 1));
        }
    }).detach();
}

void ProjectorManager::triggerSequential(bool runOnce, int delayMs) {
    std::thread([this, runOnce, delayMs]() {
        // Pro1
        {
            std::lock_guard<std::mutex> lock(mutex_);
            controller_->slotTriggerITBoard(PRO1_IDX);
            controller_->RunPatternSequence(PRO1_IDX, runOnce);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(delayMs));
        // Pro2
        {
            std::lock_guard<std::mutex> lock(mutex_);
            controller_->slotTriggerITBoard(PRO2_IDX);
            controller_->RunPatternSequence(PRO2_IDX, runOnce);
        }
        emit statusMessage(QStringLiteral("顺序投射: Pro1 → Pro2 (延迟 %1ms)").arg(delayMs));
    }).detach();
}

void ProjectorManager::triggerAll(bool runOnce) {
    std::thread([this, runOnce]() {
        std::lock_guard<std::mutex> lock(mutex_);
        bool ok1 = controller_->slotTriggerITBoard(PRO1_IDX);
        bool ok2 = controller_->slotTriggerITBoard(PRO2_IDX);
        if (ok1) controller_->RunPatternSequence(PRO1_IDX, runOnce);
        if (ok2) controller_->RunPatternSequence(PRO2_IDX, runOnce);
        if (ok1 && ok2) {
            emit statusMessage(QStringLiteral("全部投影仪同时投射"));
        } else {
            if (!ok1) emit errorOccurred(PRO1_IDX, QStringLiteral("Pro1 触发失败"));
            if (!ok2) emit errorOccurred(PRO2_IDX, QStringLiteral("Pro2 触发失败"));
        }
    }).detach();
}
