#pragma once

#include <stdexcept>
#include <string>

namespace tp {

class AppException : public std::runtime_error {
public:
    explicit AppException(const std::string& msg) : std::runtime_error(msg) {}
    ~AppException() noexcept override = default;
};

// 异常派生类生成宏
#define TP_DEFINE_EX(Name, Base) \
    class Name : public Base { \
    public: \
        explicit Name(const std::string& msg) : Base(msg) {} \
        ~Name() noexcept override = default; \
    }

// 派生异常类
TP_DEFINE_EX(CalibException,        AppException);
TP_DEFINE_EX(ImageLoadException,    AppException);
TP_DEFINE_EX(PhaseDecodeException,  AppException);
TP_DEFINE_EX(EpipolarException,     AppException);
TP_DEFINE_EX(ReconstructException,  AppException);
TP_DEFINE_EX(MeasureException,      AppException);
TP_DEFINE_EX(IOException,           AppException);
TP_DEFINE_EX(FilterException,       AppException);

} // namespace tp
