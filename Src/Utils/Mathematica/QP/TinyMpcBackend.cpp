#include "TinyMpcBackend.hpp"

#include <bit>
#include <limits>

#include "tinympc/tiny_api.hpp"

namespace {

using RowMajorF = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using VecF = Eigen::Matrix<float, Eigen::Dynamic, 1>;

// 使用有限极值表示无界，兼容 -ffast-math。
constexpr tinytype UNBOUNDED = std::numeric_limits<float>::max();

// 检查 IEEE-754 位模式，使有效性校验在 -ffast-math 下仍然有效。
bool finite(float _value)
{
    static_assert(sizeof(float) == sizeof(uint32_t) && std::numeric_limits<float>::is_iec559);
    return (std::bit_cast<uint32_t>(_value) & 0x7f800000u) != 0x7f800000u;
}

bool finiteArray(const float *_values, int _count)
{
    if (!_values)
        return false;
    for (int i = 0; i < _count; i++)
        if (!finite(_values[i]))
            return false;
    return true;
}

bool validBounds(const float *_min, const float *_max, int _count)
{
    if (!finiteArray(_min, _count) || !finiteArray(_max, _count))
        return false;
    for (int i = 0; i < _count; i++)
        if (_min[i] > _max[i])
            return false;
    return true;
}

// 上游只使用对角线；加惩罚后的权重必须保持正定，Riccati 预计算才可逆。
bool validWeights(const float *_q, const float *_r, int _nx, int _nu, float _rho)
{
    for (int i = 0; i < _nx; i++)
        if (_q[(i * _nx) + i] <= -_rho)
            return false;
    for (int i = 0; i < _nu; i++)
        if (_r[(i * _nu) + i] <= -_rho)
            return false;
    return true;
}

// 将按行存储的 float 数据转换为上游标量类型。
tinyMatrix toTiny(const float *_d, int _rows, int _cols)
{
    return Eigen::Map<const RowMajorF>(_d, _rows, _cols).cast<tinytype>();
}

// 将列向量复制到整个预测时域。
tinyMatrix broadcast(const float *_v, int _rows, int _cols)
{
    return Eigen::Map<const VecF>(_v, _rows).cast<tinytype>().replicate(1, _cols);
}

TinySolver *cast(void *_p) { return static_cast<TinySolver *>(_p); }
const TinySolver *cast(const void *_p) { return static_cast<const TinySolver *>(_p); }

// 约束开关由本层固定，避免上游默认值变化后静默失效。
void setConstraintFlags(TinySolver *_solver)
{
    auto *s = _solver->settings;
    s->en_state_bound = 1;
    s->en_input_bound = 1;
    s->en_state_soc = 0;
    s->en_input_soc = 0;
    s->en_state_linear = 0;
    s->en_input_linear = 0;
    s->en_tv_state_linear = 0;
    s->en_tv_input_linear = 0;
}

} // namespace

bool TinyMpcBackend::setup(const Dims_s &_dims, const float *_a, const float *_b, const float *_f, const float *_q,
                           const float *_r, float _rho)
{
    if (solver_ || _dims.nx <= 0 || _dims.nu <= 0 || _dims.horizon < 2 || !finite(_rho) || _rho <= 0)
        return false; // 求解器内存为永久分配，仅允许成功初始化一次。

    dims_ = _dims;
    const auto [nx, nu, n] = _dims;

    if (!finiteArray(_a, nx * nx) || !finiteArray(_b, nx * nu) || !finiteArray(_f, nx) || !finiteArray(_q, nx * nx) ||
        !finiteArray(_r, nu * nu) || !validWeights(_q, _r, nx, nu, _rho))
        return false;

    TinySolver *solver = nullptr;
    const int status = tiny_setup(&solver, toTiny(_a, nx, nx), toTiny(_b, nx, nu), toTiny(_f, nx, 1),
                                  toTiny(_q, nx, nx), toTiny(_r, nu, nu), _rho, nx, nu, n, 0);
    if (status != 0)
        return false;
    solver_ = solver;

    // 预置箱约束开关，并在调用 setBounds 前用有限极值表示无界。
    setConstraintFlags(solver);
    ready_ = tiny_set_bound_constraints(solver, tinyMatrix::Constant(nx, n, -UNBOUNDED),
                                        tinyMatrix::Constant(nx, n, UNBOUNDED),
                                        tinyMatrix::Constant(nu, n - 1, -UNBOUNDED),
                                        tinyMatrix::Constant(nu, n - 1, UNBOUNDED)) == 0;
    return ready_;
}

bool TinyMpcBackend::validSettings(const Settings_s &_settings)
{
    return _settings.maxIter > 0 && _settings.checkTermination > 0 && finite(_settings.absPriTol) &&
           _settings.absPriTol > 0 && finite(_settings.absDuaTol) && _settings.absDuaTol > 0;
}

bool TinyMpcBackend::applySettings(const Settings_s &_settings)
{
    if (!ready_ || !validSettings(_settings))
        return false;
    auto *s = cast(solver_)->settings;
    s->abs_pri_tol = _settings.absPriTol;
    s->abs_dua_tol = _settings.absDuaTol;
    s->max_iter = _settings.maxIter;
    s->check_termination = _settings.checkTermination;
    return true;
}

bool TinyMpcBackend::setBounds(const float *_xMin, const float *_xMax, const float *_uMin, const float *_uMax)
{
    if (!ready_)
        return false;
    const auto [nx, nu, n] = dims_;
    if (!validBounds(_xMin, _xMax, nx) || !validBounds(_uMin, _uMax, nu))
        return false;
    return tiny_set_bound_constraints(cast(solver_), broadcast(_xMin, nx, n), broadcast(_xMax, nx, n),
                                      broadcast(_uMin, nu, n - 1), broadcast(_uMax, nu, n - 1)) == 0;
}

bool TinyMpcBackend::setInitialState(const float *_x0)
{
    return ready_ && finiteArray(_x0, dims_.nx) && tiny_set_x0(cast(solver_), toTiny(_x0, dims_.nx, 1)) == 0;
}

bool TinyMpcBackend::setStateRef(const float *_xRef)
{
    return ready_ && finiteArray(_xRef, dims_.nx) &&
           tiny_set_x_ref(cast(solver_), broadcast(_xRef, dims_.nx, dims_.horizon)) == 0;
}

bool TinyMpcBackend::setInputRef(const float *_uRef)
{
    return ready_ && finiteArray(_uRef, dims_.nu) &&
           tiny_set_u_ref(cast(solver_), broadcast(_uRef, dims_.nu, dims_.horizon - 1)) == 0;
}

bool TinyMpcBackend::solve()
{
    if (!ready_ || tiny_solve(cast(solver_)) != 0)
        return false;
    // 收敛后仍拒绝非有限结果，避免把 NaN 控制量交给调用方。
    const auto *s = cast(solver_);
    return s->solution->u.allFinite() && s->solution->x.allFinite();
}

// 先拒绝无效读取，再访问调用方的输出。
bool TinyMpcBackend::getInput(int _k, float *_u) const
{
    if (!ready_ || !_u || _k < 0 || _k >= dims_.horizon - 1)
        return false;
    const auto &u = cast(solver_)->solution->u;
    for (int i = 0; i < dims_.nu; i++)
        _u[i] = static_cast<float>(u(i, _k));
    return true;
}

bool TinyMpcBackend::getState(int _k, float *_x) const
{
    if (!ready_ || !_x || _k < 0 || _k >= dims_.horizon)
        return false;
    const auto &x = cast(solver_)->solution->x;
    for (int i = 0; i < dims_.nx; i++)
        _x[i] = static_cast<float>(x(i, _k));
    return true;
}

int TinyMpcBackend::iterations() const { return ready_ ? cast(solver_)->solution->iter : 0; }
