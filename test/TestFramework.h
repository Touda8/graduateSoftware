#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <cmath>
#include <stdexcept>

namespace testing {
    struct TestCase { std::string name; std::function<void()> fn; bool passed = false; std::string error; };
    inline std::vector<TestCase>& cases() { static std::vector<TestCase> c; return c; }

    #define TEST(Suite, Name) \
        void Suite##_##Name(); \
        static bool reg_##Suite##_##Name = (testing::cases().push_back({#Suite "." #Name, Suite##_##Name}), true); \
        void Suite##_##Name()

    #define ASSERT_TRUE(cond) if (!(cond)) throw std::runtime_error("ASSERT_TRUE failed: " #cond)
    #define ASSERT_FALSE(cond) if (cond) throw std::runtime_error("ASSERT_FALSE failed: " #cond)
    #define ASSERT_EQ(a, b) if ((a) != (b)) throw std::runtime_error("ASSERT_EQ failed")
    #define ASSERT_GT(a, b) if (!((a) > (b))) throw std::runtime_error("ASSERT_GT failed")
    #define ASSERT_LT(a, b) if (!((a) < (b))) throw std::runtime_error("ASSERT_LT failed")
    #define ASSERT_NEAR(a, b, tol) if (std::abs((a)-(b)) > (tol)) throw std::runtime_error("ASSERT_NEAR failed")
    #define ASSERT_THROW(expr, ExType) { bool caught = false; try { expr; } catch(const ExType&) { caught = true; } catch(...) {} if (!caught) throw std::runtime_error("ASSERT_THROW failed: " #ExType " not thrown"); }
    #define ASSERT_NO_THROW(expr) { try { expr; } catch(const std::exception& e) { throw std::runtime_error(std::string("ASSERT_NO_THROW failed: ") + e.what()); } }

    inline int runAll() {
        int passed = 0, failed = 0;
        for (auto& tc : cases()) {
            try { tc.fn(); tc.passed = true; ++passed; std::cout << "[PASS] " << tc.name << "\n"; }
            catch (const std::exception& e) { tc.error = e.what(); ++failed; std::cout << "[FAIL] " << tc.name << ": " << e.what() << "\n"; }
        }
        std::cout << "\n" << passed << " passed, " << failed << " failed, " << (passed+failed) << " total\n";
        return failed > 0 ? 1 : 0;
    }
}
