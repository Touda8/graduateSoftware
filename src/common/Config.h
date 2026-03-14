#pragma once

#include <QWidget>
#include <QString>
#include <filesystem>

namespace tp {

class Config {
public:
    static Config& instance();

    void save(QWidget* root, const std::filesystem::path& path = "config/params.json");
    void load(QWidget* root, const std::filesystem::path& path = "config/params.json");

private:
    Config() = default;
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;
};

} // namespace tp
