#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "Task.hpp"
#include "TinyMPC.hpp"
#include "queue.h"

// Construct after HAL initialization. Once started, keep this object alive for the application lifetime.
template <int nx, int nu, int horizon, configSTACK_DEPTH_TYPE stackWords> class MPCTask final {
public:
    using Settings_s = typename TinyMPC<nx, nu, horizon>::Settings_s;

    struct Sample_s {
        TickType_t tick{};
        std::uint32_t sequence{};
        std::uint32_t epoch{};
    };

    struct Request_s {
        std::array<float, nx> x{};
        std::array<float, nx> xRef{};
        std::array<float, nu> uRef{};
        Sample_s sample{};
    };

    enum class Status_e : std::uint8_t { NO_RESULT, SUCCESS, REQUEST_REJECTED, SOLVE_FAILED, INVALID_OUTPUT };

    struct Result_s {
        std::array<float, nu> u{};
        Sample_s sample{};
        TickType_t elapsedTicks{};
        Status_e status{ Status_e::NO_RESULT };
    };

    MPCTask(const Matrix<nx, nx> &_a, const Matrix<nx, nu> &_b, const Matrix<nx, nx> &_q, const Matrix<nu, nu> &_r,
            float _rho, const Matrix<nx, 1> &_f = Matrix<nx, 1>::zeros(), const Settings_s &_settings = {})
            : solver_(_a, _b, _q, _r, _rho, _f, _settings)
    {}

    MPCTask(const MPCTask &) = delete;
    MPCTask &operator=(const MPCTask &) = delete;

    // Configure and start from a single initialization context before publishing the instance to the APP.
    bool setBounds(const Matrix<nx, 1> &_xMin, const Matrix<nx, 1> &_xMax, const Matrix<nu, 1> &_uMin,
                   const Matrix<nu, 1> &_uMax)
    {
        return task_ == nullptr && solver_.setBounds(_xMin, _xMax, _uMin, _uMax);
    }

    bool start(TaskPriority_e _priority = TaskPriority_e::LOW1)
    {
        const auto priority = static_cast<UBaseType_t>(_priority);
        if (task_ != nullptr || !solver_.ready() || priority < static_cast<UBaseType_t>(TaskPriority_e::LOW1) ||
            priority >= static_cast<UBaseType_t>(TaskPriority_e::HIGH1) || priority >= configMAX_PRIORITIES)
            return false;

        requests_ = xQueueCreateStatic(1, sizeof(Request_s), requestStorage_, &requestQueue_);
        results_ = xQueueCreateStatic(1, sizeof(Result_s), resultStorage_, &resultQueue_);
        if (requests_ == nullptr || results_ == nullptr)
            return false;

        task_ = xTaskCreateStatic([](void *_context) { static_cast<MPCTask *>(_context)->run(); }, "MPCTask",
                                  stackWords, this, priority, stack_, &taskStorage_);
        return task_ != nullptr;
    }

    // Task context only. A write replaces the pending request but does not interrupt an active solve.
    bool submit(const Request_s &_request) { return task_ != nullptr && xQueueOverwrite(requests_, &_request) == pdPASS; }

    // A single APP consumer takes the latest unread result without waiting, including failures.
    bool tryRead(Result_s &_result) { return task_ != nullptr && xQueueReceive(results_, &_result, 0) == pdPASS; }

private:
    static_assert(stackWords >= configMINIMAL_STACK_SIZE);
    static_assert(std::is_trivially_copyable_v<Request_s> && std::is_trivially_copyable_v<Result_s>);
    static_assert(std::numeric_limits<float>::is_iec559 && sizeof(float) == sizeof(std::uint32_t));

    template <int size> static Matrix<size, 1> toMatrix(const std::array<float, size> &_values)
    {
        Matrix<size, 1> matrix;
        for (int i = 0; i < size; ++i)
            matrix(i, 0) = _values[i];
        return matrix;
    }

    Status_e solveRequest(const Request_s &_request, Result_s &_result)
    {
        if (!solver_.setInitialState(toMatrix<nx>(_request.x)) || !solver_.setStateRef(toMatrix<nx>(_request.xRef)) ||
            !solver_.setInputRef(toMatrix<nu>(_request.uRef)))
            return Status_e::REQUEST_REJECTED;
        if (!solver_.solve())
            return Status_e::SOLVE_FAILED;

        Matrix<nu, 1> input;
        if (!solver_.getInput(input))
            return Status_e::INVALID_OUTPUT;
        std::array<float, nu> control;
        for (int i = 0; i < nu; ++i) {
            const float value = input(i, 0);
            // Check after double-to-float conversion, including builds using fast-math.
            if ((std::bit_cast<std::uint32_t>(value) & 0x7f800000U) == 0x7f800000U)
                return Status_e::INVALID_OUTPUT;
            control[i] = value;
        }
        _result.u = control;
        return Status_e::SUCCESS;
    }

    [[noreturn]] void run()
    {
        for (;;) {
            Request_s request;
            if (xQueueReceive(requests_, &request, portMAX_DELAY) != pdPASS)
                continue;

            Result_s result{ .sample = request.sample };
            const auto start = xTaskGetTickCount();
            result.status = solveRequest(request, result);
            result.elapsedTicks = static_cast<TickType_t>(xTaskGetTickCount() - start);
            xQueueOverwrite(results_, &result);
        }
    }

    TinyMPC<nx, nu, horizon> solver_;
    std::uint8_t requestStorage_[sizeof(Request_s)];
    std::uint8_t resultStorage_[sizeof(Result_s)];
    StaticQueue_t requestQueue_{};
    StaticQueue_t resultQueue_{};
    StackType_t stack_[stackWords];
    StaticTask_t taskStorage_{};
    QueueHandle_t requests_{};
    QueueHandle_t results_{};
    TaskHandle_t task_{};
};
