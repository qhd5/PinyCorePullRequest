/**
 * @file TinyMPC.hpp
 * @brief 基于 Matrix 的 ADMM 求解器接口，使用补丁后的上游实现（Eigen、double）。
 *
 * 使用对角阶段权重 Q/R 和上游稳态终端缓存。
 * 每个实例独占求解器的永久内存。应在 HAL 初始化后构造，
 * 并让实例存活至所属应用结束。
 */

#pragma once

#include "Matrix.hpp"
#include "TinyMpcBackend.hpp"

template <int nx, int nu, int horizon> class TinyMPC final {
    static_assert(nx > 0 && nu > 0 && horizon >= 2, "invalid TinyMPC dimensions");

public:
    using Settings_s = TinyMpcBackend::Settings_s;

    // A(nx,nx)、B(nx,nu)，Q/R 只使用对角线且加 rho 后必须为正，rho 为正数，f 为仿射偏置。
    // 设置或模型数据无效时，求解器保持未就绪状态。

    TinyMPC(const Matrix<nx, nx> &_a, const Matrix<nx, nu> &_b, const Matrix<nx, nx> &_q, const Matrix<nu, nu> &_r,
            float _rho, const Matrix<nx, 1> &_f = Matrix<nx, 1>::zeros(), const Settings_s &_settings = {})
    {
        if (TinyMpcBackend::validSettings(_settings) &&
            backend_.setup({ nx, nu, horizon }, ptr(_a), ptr(_b), ptr(_f), ptr(_q), ptr(_r), _rho))
            backend_.applySettings(_settings);
    }

    // 每个求解器独占上游内存，不可复制。
    TinyMPC(const TinyMPC &) = delete;
    TinyMPC &operator=(const TinyMPC &) = delete;

    // 表示构造成功，不代表求解已收敛。
    bool ready() const { return backend_.ready(); }

    bool setSettings(const Settings_s &_settings) { return backend_.applySettings(_settings); }

    // 整个预测时域使用相同的箱约束；默认用有限极值表示无界。
    bool setBounds(const Matrix<nx, 1> &_xMin, const Matrix<nx, 1> &_xMax, const Matrix<nu, 1> &_uMin,
                   const Matrix<nu, 1> &_uMax)
    {
        return backend_.setBounds(ptr(_xMin), ptr(_xMax), ptr(_uMin), ptr(_uMax));
    }

    bool setInitialState(const Matrix<nx, 1> &_x0) { return backend_.setInitialState(ptr(_x0)); }

    // 参考值在整个预测时域内保持不变。
    bool setStateRef(const Matrix<nx, 1> &_xRef) { return backend_.setStateRef(ptr(_xRef)); }
    bool setInputRef(const Matrix<nu, 1> &_uRef) { return backend_.setInputRef(ptr(_uRef)); }

    // 从上次迭代结果热启动；仅在收敛且结果有限时返回 true。
    bool solve() { return backend_.solve(); }

    // 带检查的读取在失败时保留输出。N 个状态节点对应 N-1 个控制量。
    bool getInput(Matrix<nu, 1> &_u, int _k = 0) const { return backend_.getInput(_k, _u.arm_mat_.pData); }
    bool getState(Matrix<nx, 1> &_x, int _k) const { return backend_.getState(_k, _x.arm_mat_.pData); }

    // 便捷读取在索引无效或求解器未就绪时返回零向量。
    Matrix<nu, 1> input(int _k = 0) const
    {
        Matrix<nu, 1> u;
        backend_.getInput(_k, u.arm_mat_.pData);
        return u;
    }

    // 读取第 k 个预测状态；读取无效时返回零向量。
    Matrix<nx, 1> state(int _k) const
    {
        Matrix<nx, 1> x;
        backend_.getState(_k, x.arm_mat_.pData);
        return x;
    }

    // 最近一次求解的迭代次数。
    int iterations() const { return backend_.iterations(); }

private:
    TinyMpcBackend backend_;

    template <int rows, int cols> static const float *ptr(const Matrix<rows, cols> &_m) { return _m.arm_mat_.pData; }
};
