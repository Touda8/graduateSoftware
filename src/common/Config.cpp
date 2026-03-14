#include "Config.h"
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>

namespace tp {

Config& Config::instance() {
    static Config cfg;
    return cfg;
}

void Config::save(QWidget* root, const std::filesystem::path& path) {
    if (!root) return;

    std::filesystem::create_directories(path.parent_path());

    QJsonObject obj;

    const auto spinBoxes = root->findChildren<QSpinBox*>();
    for (auto* w : spinBoxes) {
        if (!w->objectName().isEmpty())
            obj[w->objectName()] = w->value();
    }

    const auto dblSpinBoxes = root->findChildren<QDoubleSpinBox*>();
    for (auto* w : dblSpinBoxes) {
        if (!w->objectName().isEmpty())
            obj[w->objectName()] = w->value();
    }

    const auto lineEdits = root->findChildren<QLineEdit*>();
    for (auto* w : lineEdits) {
        if (!w->objectName().isEmpty())
            obj[w->objectName()] = w->text();
    }

    const auto combos = root->findChildren<QComboBox*>();
    for (auto* w : combos) {
        if (!w->objectName().isEmpty())
            obj[w->objectName()] = w->currentIndex();
    }

    const auto checks = root->findChildren<QCheckBox*>();
    for (auto* w : checks) {
        if (!w->objectName().isEmpty())
            obj[w->objectName()] = w->isChecked();
    }

    QFile file(QString::fromStdString(path.string()));
    if (file.open(QIODevice::WriteOnly)) {
        file.write(QJsonDocument(obj).toJson());
    }
}

void Config::load(QWidget* root, const std::filesystem::path& path) {
    if (!root) return;

    QFile file(QString::fromStdString(path.string()));
    if (!file.open(QIODevice::ReadOnly)) return;

    auto doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject()) return;
    auto obj = doc.object();

    for (auto it = obj.begin(); it != obj.end(); ++it) {
        auto* widget = root->findChild<QWidget*>(it.key());
        if (!widget) continue;

        if (auto* sb = qobject_cast<QSpinBox*>(widget)) {
            sb->setValue(it.value().toInt());
        } else if (auto* dsb = qobject_cast<QDoubleSpinBox*>(widget)) {
            dsb->setValue(it.value().toDouble());
        } else if (auto* le = qobject_cast<QLineEdit*>(widget)) {
            le->setText(it.value().toString());
        } else if (auto* cb = qobject_cast<QComboBox*>(widget)) {
            cb->setCurrentIndex(it.value().toInt());
        } else if (auto* chk = qobject_cast<QCheckBox*>(widget)) {
            chk->setChecked(it.value().toBool());
        }
    }
}

} // namespace tp
