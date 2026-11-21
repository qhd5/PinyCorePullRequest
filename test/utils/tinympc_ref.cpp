// 使用上游 TinyMPC（Eigen、double）进行参考求解。放在独立编译单元中，
// 避免上游全局的 `using namespace Eigen` 与框架 Matrix 类型冲突。
#include "tinympc_ref.hpp"
#include "tinympc/tiny_api.hpp"

int tinympcRefSolve(const TinyRefProblem_s &_p, double *_u0, int *_iter)
{
    const int nx = _p.nx, nu = _p.nu, n = _p.horizon;
    tinyMatrix a = Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(_p.a, nx, nx);
    tinyMatrix b = Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(_p.b, nx, nu);
    tinyMatrix f = tinyMatrix::Zero(nx, 1);
    tinyMatrix q = Map<const tinyVector>(_p.q, nx).asDiagonal();
    tinyMatrix r = Map<const tinyVector>(_p.r, nu).asDiagonal();

    TinySolver *solver = nullptr;
    if (tiny_setup(&solver, a, b, f, q, r, _p.rho, nx, nu, n, 0) != 0)
        return -1;

    tinyMatrix xMin = Map<const tinyVector>(_p.xMin, nx).replicate(1, n);
    tinyMatrix xMax = Map<const tinyVector>(_p.xMax, nx).replicate(1, n);
    tinyMatrix uMin = Map<const tinyVector>(_p.uMin, nu).replicate(1, n - 1);
    tinyMatrix uMax = Map<const tinyVector>(_p.uMax, nu).replicate(1, n - 1);
    tiny_set_bound_constraints(solver, xMin, xMax, uMin, uMax);

    solver->settings->max_iter = _p.maxIter;
    solver->settings->abs_pri_tol = _p.tol;
    solver->settings->abs_dua_tol = _p.tol;

    tiny_set_x0(solver, Map<const tinyVector>(_p.x0, nx));
    tiny_set_x_ref(solver, Map<const tinyVector>(_p.xRef, nx).replicate(1, n));

    const int status = tiny_solve(solver);
    for (int i = 0; i < nu; i++)
        _u0[i] = solver->solution->u(i, 0);
    *_iter = solver->solution->iter;
    return status;
}
