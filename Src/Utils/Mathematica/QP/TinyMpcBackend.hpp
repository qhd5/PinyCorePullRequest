/**
 * @file TinyMpcBackend.hpp
 * @brief 基于不透明指针的 float 数组适配层；Eigen 类型仅在实现中使用。
 */

#pragma once

#include <cstdint>

class TinyMpcBackend final {
public:
    struct Dims_s {
        int nx;      // 状态维度
        int nu;      // 输入维度
        int horizon; // 状态节点数（比控制量个数多一）
    };

    struct Settings_s {
        float absPriTol = 1e-3f;       // 原始残差容差，必须为正数
        float absDuaTol = 1e-3f;       // 对偶残差容差，必须为正数
        uint16_t maxIter = 100;        // 最大迭代次数，必须大于零
        uint16_t checkTermination = 1; // 终止条件检查间隔，必须大于零
    };

    TinyMpcBackend() = default;
    TinyMpcBackend(const TinyMpcBackend &) = delete;
    TinyMpcBackend &operator=(const TinyMpcBackend &) = delete;

    // 所有矩阵均按行存储。Q/R 只使用对角线，且加 rho 后必须为正。
    // 求解器分配的内存保留至应用结束。
    bool setup(const Dims_s &_dims, const float *_a, const float *_b, const float *_f, const float *_q, const float *_r,
               float _rho);
    // 无效更新保留原有设置。
    static bool validSettings(const Settings_s &_settings);
    bool applySettings(const Settings_s &_settings);
    bool ready() const { return ready_; }

    // 约束边界和参考值在整个预测时域内保持不变。
    bool setBounds(const float *_xMin, const float *_xMax, const float *_uMin, const float *_uMax);
    bool setInitialState(const float *_x0);
    bool setStateRef(const float *_xRef);
    bool setInputRef(const float *_uRef);

    // 仅当收敛且结果有限时返回 true。
    bool solve();

    // 带检查的读取在失败时保留输出；读取成功不代表求解已收敛。
    bool getInput(int _k, float *_u) const;
    bool getState(int _k, float *_x) const;
    int iterations() const;

private:
    void *solver_ = nullptr; // 使用不透明 TinySolver 指针，避免暴露 Eigen 类型。
    Dims_s dims_{};
    bool ready_ = false;
};
