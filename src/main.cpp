#include "MainWindow.h"
#include <QApplication>
#include <QSurfaceFormat>
#include <QVTKOpenGLNativeWidget.h>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <iostream>
#include <clocale>
#include <cstdio>

#ifdef _WIN32
#include <windows.h>
#endif

// 全局日志文件
static QFile* g_logFile = nullptr;

static void fileMessageHandler(QtMsgType type, const QMessageLogContext& ctx, const QString& msg) {
    Q_UNUSED(ctx);
    if (!g_logFile || !g_logFile->isOpen()) return;
    const char* tag = "DEBUG";
    switch (type) {
        case QtWarningMsg:  tag = "WARN"; break;
        case QtCriticalMsg: tag = "CRIT"; break;
        case QtFatalMsg:    tag = "FATAL"; break;
        default: break;
    }
    QTextStream ts(g_logFile);
    ts << QDateTime::currentDateTime().toString("[hh:mm:ss.zzz] ")
       << tag << ": " << msg << "\n";
    ts.flush();
}

int main(int argc, char* argv[]) {
    // Windows: 让 CRT 使用 UTF-8
    std::setlocale(LC_ALL, ".UTF-8");

    // 用 argv[0] 推出 exe 所在目录，不依赖 QApplication
    QString exeDir = QFileInfo(QString::fromLocal8Bit(argv[0])).absolutePath();
    // 项目根目录 = exe/../.. (因为 exe 在 build/Release/)
    QString projRoot = QDir(exeDir + "/../..").absolutePath();
    QString logDir = projRoot + "/logs";
    QDir().mkpath(logDir);

#ifdef _WIN32
    // 把 stdout/stderr 重定向到日志文件
    {
        QString stdoutPath = logDir + "/stdout.log";
        freopen(stdoutPath.toLocal8Bit().constData(), "w", stdout);
        freopen(stdoutPath.toLocal8Bit().constData(), "a", stderr);
    }
#endif

    QString logPath = logDir + "/app_debug.log";

    g_logFile = new QFile(logPath);
    g_logFile->open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text);
    qInstallMessageHandler(fileMessageHandler);

    fprintf(stderr, "[STARTUP] Log path: %s\n", logPath.toLocal8Bit().constData());
    fprintf(stderr, "[STARTUP] Log open: %s\n", g_logFile->isOpen() ? "YES" : "NO");

    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication app(argc, argv);

    std::cout << "[STARTUP] Qt version: " << qVersion() << std::endl;
    std::cout << "[STARTUP] App dir: " << QCoreApplication::applicationDirPath().toStdString() << std::endl;
    qDebug() << "[STARTUP] Log file active, Qt version:" << qVersion();

    std::cout << "[STARTUP] Constructing MainWindow..." << std::endl;
    MainWindow w;
    std::cout << "[STARTUP] MainWindow created, calling show()..." << std::endl;
    w.show();
    std::cout << "[STARTUP] Window shown, entering event loop." << std::endl;
    qDebug() << "[STARTUP] Event loop starting";

    int ret = app.exec();

    delete g_logFile;
    g_logFile = nullptr;
    return ret;
}
