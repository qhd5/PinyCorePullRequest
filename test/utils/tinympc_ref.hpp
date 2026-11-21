#pragma once

// 封装实现与上游参考实现共用的纯数组问题描述。
struct TinyRefProblem_s {
    int nx, nu, horizon;
    const double *a;    // 长度为 nx * nx，按行存储
    const double *b;    // 长度为 nx * nu，按行存储
    const double *q;    // 长度为 nx
    const double *r;    // 长度为 nu
    const double *xMin; // 长度为 nx
    const double *xMax; // 长度为 nx
    const double *uMin; // 长度为 nu
    const double *uMax; // 长度为 nu
    const double *x0;   // 长度为 nx
    const double *xRef; // 长度为 nx
    double rho;
    int maxIter;
    double tol;
};

// 返回上游 tiny_solve 状态（0 表示收敛），并写入第一个控制量和迭代次数。
int tinympcRefSolve(const TinyRefProblem_s &_p, double *_u0, int *_iter);
