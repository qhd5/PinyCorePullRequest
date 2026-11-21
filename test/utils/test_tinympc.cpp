#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "TinyMPC.hpp"
#include "tinympc_ref.hpp"

#include <iostream>
#include <limits>
#include <sstream>

namespace {

// 双积分器，dt = 0.1
constexpr int NX = 2, NU = 1, HORIZON = 10;
constexpr double A[] = { 1, 0.1, 0, 1 };
constexpr double B[] = { 0.005, 0.1 };
constexpr double Q[] = { 10, 1 };
constexpr double R[] = { 0.1 };
constexpr double X_MIN[] = { -5, -5 }, X_MAX[] = { 5, 5 };
constexpr double U_MIN[] = { -2 }, U_MAX[] = { 2 };
constexpr double X0[] = { 1, 0 }, X_REF[] = { 0, 0 };
constexpr double RHO = 1.0, TOL = 1e-3;
constexpr int MAX_ITER = 200;

TinyMpcBackend::Settings_s testSettings()
{
    TinyMpcBackend::Settings_s settings;
    settings.maxIter = MAX_ITER;
    settings.absPriTol = settings.absDuaTol = TOL;
    return settings;
}

// 采用与 APP 层成员相同的构造方式
struct Planner_s {
    TinyMPC<NX, NU, HORIZON> mpc{ { { 1, 0.1f }, { 0, 1 } },
                                  { { 0.005f }, { 0.1f } },
                                  { { 10, 0 }, { 0, 1 } },
                                  { { 0.1f } },
                                  RHO,
                                  Matrix<NX, 1>::zeros(),
                                  testSettings() };

    bool configure()
    {
        return mpc.ready() && mpc.setBounds({ { -5 }, { -5 } }, { { 5 }, { 5 } }, { { -2 } }, { { 2 } }) &&
               mpc.setInitialState({ { 1 }, { 0 } }) && mpc.setStateRef({ { 0 }, { 0 } });
    }
};

} // 匿名命名空间

TEST_CASE("tinympc wrapper matches upstream", "[tinympc]")
{
    const TinyRefProblem_s prob{
        NX, NU, HORIZON, A, B, Q, R, X_MIN, X_MAX, U_MIN, U_MAX, X0, X_REF, RHO, MAX_ITER, TOL
    };
    double refU0 = 0;
    int refIter = 0;
    REQUIRE(tinympcRefSolve(prob, &refU0, &refIter) == 0);

    Planner_s planner;
    REQUIRE(planner.configure());
    REQUIRE(planner.mpc.solve());
    REQUIRE_THAT(planner.mpc.input()(0, 0), Catch::Matchers::WithinAbs(refU0, 1e-4));
    REQUIRE(planner.mpc.iterations() == refIter);
}

TEST_CASE("tinympc instances are independent", "[tinympc]")
{
    Planner_s a, b;
    REQUIRE(a.configure());
    REQUIRE(b.configure());
    REQUIRE(b.mpc.setInitialState({ { -1 }, { 0 } }));
    REQUIRE(a.mpc.solve());
    REQUIRE(b.mpc.solve());
    REQUIRE_THAT(a.mpc.input()(0, 0), Catch::Matchers::WithinAbs(-b.mpc.input()(0, 0), 1e-4));
}

TEST_CASE("tinympc wrapper respects input bounds", "[tinympc]")
{
    Planner_s planner;
    REQUIRE(planner.configure());
    REQUIRE(planner.mpc.solve());
    const float u0 = planner.mpc.input()(0, 0);
    REQUIRE(u0 <= 2.f + 1e-3f);
    REQUIRE(u0 >= -2.f - 1e-3f);
    REQUIRE(u0 < 0.f); // 使 x 回到原点
}

TEST_CASE("tinympc closed-loop simulation", "[tinympc]")
{
    // 滚动时域：求解，将 u0 施加到被控对象，然后重复
    Planner_s planner;
    REQUIRE(planner.configure());
    float pos = 1.f, vel = 0.f;
    for (int t = 0; t < 100; t++) {
        REQUIRE(planner.mpc.setInitialState({ { pos }, { vel } }));
        REQUIRE(planner.mpc.solve());
        const float u = planner.mpc.input()(0, 0);
        REQUIRE(u <= 2.f + 1e-3f);
        REQUIRE(u >= -2.f - 1e-3f);
        pos += 0.1f * vel + 0.005f * u;
        vel += 0.1f * u;
        if (t % 10 == 0)
            printf("t=%3d pos=% .4f vel=% .4f u=% .4f iter=%d\n", t, pos, vel, u, planner.mpc.iterations());
    }
    REQUIRE(std::fabs(pos) < 1e-2f);
    REQUIRE(std::fabs(vel) < 1e-2f);
}

TEST_CASE("tinympc rejects invalid bounds without replacing valid bounds", "[tinympc][validation]")
{
    Planner_s planner;
    REQUIRE(planner.configure());
    REQUIRE_FALSE(planner.mpc.setBounds({ { -5 }, { -5 } }, { { 5 }, { 5 } }, { { 2 } }, { { -2 } }));
    REQUIRE_FALSE(planner.mpc.setBounds({ { 5 }, { -5 } }, { { -5 }, { 5 } }, { { -2 } }, { { 2 } }));
    REQUIRE(planner.mpc.solve());
    REQUIRE(planner.mpc.input()(0, 0) < 0.f);
}

TEST_CASE("tinympc matches an independent scalar tracking optimum", "[tinympc][analytic]")
{
    // A=0 使 u0 与终端状态解耦。原始阶段代价为
    // 0.5*u0^2 + 1.5*(u0-1)^2，其唯一最小值点为 u0=3/4。
    for (float rho : { 0.1f, 1.f, 10.f }) {
        TinyMPC<1, 1, 3>::Settings_s settings;
        settings.maxIter = 2000;
        settings.absPriTol = settings.absDuaTol = 1e-6f;
        TinyMPC<1, 1, 3> mpc({ { 0 } }, { { 1 } }, { { 1 } }, { { 3 } }, rho, Matrix<1, 1>::zeros(), settings);
        REQUIRE(mpc.ready());
        REQUIRE(mpc.setInitialState({ { 0 } }));
        REQUIRE(mpc.setInputRef({ { 1 } }));
        REQUIRE(mpc.solve());
        INFO("rho=" << rho);
        REQUIRE_THAT(mpc.input()(0, 0), Catch::Matchers::WithinAbs(0.75, 1e-4));
    }
}

TEST_CASE("tinympc rejects invalid settings atomically", "[tinympc][validation]")
{
    Planner_s planner;
    REQUIRE(planner.configure());
    const auto original = testSettings();
    auto invalid = original;
    invalid.checkTermination = 0;
    REQUIRE_FALSE(planner.mpc.setSettings(invalid));
    invalid = original;
    invalid.maxIter = 0;
    REQUIRE_FALSE(planner.mpc.setSettings(invalid));
    for (float bad : { 0.f, -1.f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN() }) {
        invalid = original;
        invalid.absPriTol = bad;
        REQUIRE_FALSE(planner.mpc.setSettings(invalid));
        invalid = original;
        invalid.absDuaTol = bad;
        REQUIRE_FALSE(planner.mpc.setSettings(invalid));
    }
    REQUIRE(planner.mpc.solve());
    REQUIRE(planner.mpc.setSettings(original));

    invalid = original;
    invalid.checkTermination = 0;
    TinyMPC<1, 1, 2> unready({ { 0 } }, { { 1 } }, { { 1 } }, { { 1 } }, 1.f, Matrix<1, 1>::zeros(), invalid);
    REQUIRE_FALSE(unready.ready());
    REQUIRE_FALSE(unready.solve());
    REQUIRE_FALSE(unready.setSettings(original));
    Matrix<1, 1> output{ { 42 } };
    REQUIRE_FALSE(unready.getInput(output));
    REQUIRE(output(0, 0) == 42.f);
    REQUIRE(unready.input()(0, 0) == 0.f);
}

TEST_CASE("tinympc checks prediction indices and preserves output on failure", "[tinympc][validation]")
{
    Planner_s planner;
    REQUIRE(planner.configure());
    REQUIRE(planner.mpc.solve());
    Matrix<NU, 1> u{ { 42 } };
    REQUIRE_FALSE(planner.mpc.getInput(u, -1));
    REQUIRE_FALSE(planner.mpc.getInput(u, HORIZON - 1));
    REQUIRE(u(0, 0) == 42.f);
    REQUIRE(planner.mpc.getInput(u, HORIZON - 2));
    REQUIRE(std::fabs(u(0, 0)) <= 2.f + 1e-3f);
    Matrix<NX, 1> x{ { 42 }, { 43 } };
    REQUIRE_FALSE(planner.mpc.getState(x, -1));
    REQUIRE_FALSE(planner.mpc.getState(x, HORIZON));
    REQUIRE(x(0, 0) == 42.f);
    REQUIRE(x(1, 0) == 43.f);
    REQUIRE(planner.mpc.getState(x, 0));
    REQUIRE_THAT(x(0, 0), Catch::Matchers::WithinAbs(1.f, 1e-3));
    REQUIRE(planner.mpc.getState(x, HORIZON - 1));
    REQUIRE(planner.mpc.input(-1)(0, 0) == 0.f);
    REQUIRE(planner.mpc.state(HORIZON)(1, 0) == 0.f);
}

TEST_CASE("tinympc rejects nonfinite data and invalid penalties", "[tinympc][validation]")
{
    Planner_s planner;
    REQUIRE(planner.configure());
    for (float bad : { std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                       std::numeric_limits<float>::quiet_NaN() }) {
        REQUIRE_FALSE(planner.mpc.setBounds({ { -5 }, { -5 } }, { { 5 }, { 5 } }, { { -2 } }, { { bad } }));
        REQUIRE_FALSE(planner.mpc.setBounds({ { bad }, { -5 } }, { { 5 }, { 5 } }, { { -2 } }, { { 2 } }));
        REQUIRE_FALSE(planner.mpc.setInitialState({ { bad }, { 0 } }));
        REQUIRE_FALSE(planner.mpc.setStateRef({ { bad }, { 0 } }));
        REQUIRE_FALSE(planner.mpc.setInputRef({ { bad } }));
    }
    REQUIRE(planner.mpc.solve());
    for (float bad : { 0.f, -1.f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN() }) {
        TinyMPC<1, 1, 2> invalid({ { 0 } }, { { 1 } }, { { 1 } }, { { 1 } }, bad);
        REQUIRE_FALSE(invalid.ready());
        REQUIRE_FALSE(invalid.solve());
    }
}

TEST_CASE("tinympc reports iteration exhaustion", "[tinympc][validation]")
{
    Planner_s planner;
    REQUIRE(planner.configure());
    auto settings = testSettings();
    settings.maxIter = 1;
    settings.absPriTol = settings.absDuaTol = 1e-8f;
    REQUIRE(planner.mpc.setSettings(settings));
    REQUIRE_FALSE(planner.mpc.solve());
    REQUIRE(planner.mpc.iterations() == 1);
}

TEST_CASE("tinympc solver honors the solve logging option", "[tinympc][logging]")
{
    Planner_s planner;
    REQUIRE(planner.configure());
    std::ostringstream log;
    // 始终恢复 cout，包括 REQUIRE 断言失败的情况。
    struct Restore_s {
        std::streambuf *buffer;
        ~Restore_s() { std::cout.rdbuf(buffer); }
    } restore{ std::cout.rdbuf(log.rdbuf()) };
    REQUIRE(planner.mpc.solve());
#if defined(PINY_TINYMPC_ENABLE_SOLVE_LOG)
    REQUIRE(log.str().find("Solver converged in ") != std::string::npos);
#else
    REQUIRE(log.str().empty());
#endif
}

TEST_CASE("tinympc tracks affine multi-input dynamics", "[tinympc][analytic]")
{
    TinyMPC<2, 2, 3>::Settings_s settings;
    settings.maxIter = 2000;
    settings.absPriTol = settings.absDuaTol = 1e-6f;
    TinyMPC<2, 2, 3> mpc({ { 0, 0 }, { 0, 0 } }, { { 1, 0 }, { 0, 1 } }, { { 1, 0 }, { 0, 2 } }, { { 3, 0 }, { 0, 6 } },
                         1.f, { { 0.25f }, { -0.5f } }, settings);
    REQUIRE(mpc.ready());
    REQUIRE(mpc.setInitialState({ { 0 }, { 0 } }));
    REQUIRE(mpc.setStateRef({ { 0.5f }, { -1 } }));
    REQUIRE(mpc.setInputRef({ { 1 }, { -1 } }));
    REQUIRE(mpc.solve());
    // 各通道独立：u0=(Q*(xref-f)+R*uref)/(Q+R)。
    const auto u = mpc.input();
    REQUIRE_THAT(u(0, 0), Catch::Matchers::WithinAbs(0.8125, 1e-4));
    REQUIRE_THAT(u(1, 0), Catch::Matchers::WithinAbs(-0.875, 1e-4));
    const auto x = mpc.state(1);
    REQUIRE_THAT(x(0, 0), Catch::Matchers::WithinAbs(u(0, 0) + 0.25, 1e-4));
    REQUIRE_THAT(x(1, 0), Catch::Matchers::WithinAbs(u(1, 0) - 0.5, 1e-4));
}

TEST_CASE("tinympc accepts equal bounds at the minimum horizon", "[tinympc][validation]")
{
    TinyMPC<1, 1, 2> mpc({ { 0 } }, { { 1 } }, { { 1 } }, { { 1 } }, 1.f);
    REQUIRE(mpc.ready());
    REQUIRE(mpc.setBounds({ { -5 } }, { { 5 } }, { { 0 } }, { { 0 } }));
    REQUIRE(mpc.setInitialState({ { 0 } }));
    REQUIRE(mpc.solve());
    Matrix<1, 1> output;
    REQUIRE(mpc.getInput(output, 0));
    REQUIRE(output(0, 0) == 0.f);
    REQUIRE_FALSE(mpc.getInput(output, 1));
    REQUIRE(mpc.getState(output, 1));
    REQUIRE_FALSE(mpc.getState(output, 2));
}
