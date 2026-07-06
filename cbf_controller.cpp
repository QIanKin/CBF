#include "cbf_controller.hpp"
#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Sparse>
#include <vector>

namespace controller {

namespace {

constexpr double kInf = 1e30;
constexpr double kEps = 1e-9;

// 优化形式为： 1/2 * x^T * P * x + q^T * x 不等式约束为 low <= Ax <= up
// min 0.5 * (u - u_nom)^T * W_cost * (u - u_nom) + 0.5 * ρ * δ^2
// 等价于 1/2 * u^T * W_cost * u - u_nom^T * W_cost * u + 1/2 * u_nom^T * u_nom 不等式约束为 low <= Au <= up

bool solveQP(const CBFParams& p,
             const Eigen::Vector3d& u_nom,
             const std::vector<Eigen::Vector2d>& nearest_obs,
             Eigen::Vector3d& u_out,
             bool with_slack,
             double& slack_out
            ){
    const int m = static_cast<int>(nearest_obs.size()); //障碍物约束矩阵的维度
    const int nx = with_slack ? 4 : 3;  //优化变量x的维度
    const int nr = m + nx; //约束矩阵A的行维度
    
    //构造矩阵P
    Eigen::SparseMatrix<double> P(nx, nx);
    std::vector<Eigen::Triplet<double>> Ptrip;
    for(int i = 0; i < 3; ++i)  Ptrip.emplace_back(i, i, p.w_cost(i));
    if(with_slack) Ptrip.emplace_back(3, 3, p.w_slack);
    P.setFromTriplets(Ptrip.begin(), Ptrip.end());
    
    //构造线性项q^T
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nx);
    for (int i = 0 ; i < 3; ++i) q(i) = - p.w_cost(i) * u_nom(i); //slack_nom默认为0

    //构造约束矩阵A
    Eigen::SparseMatrix<double> A(nr,nx);
    std::vector<Eigen::Triplet<double>> Atrip;
    
    //构造约束边界向量
    Eigen::VectorXd low(nr), up(nr);
    int row = 0;
    for(int i = 0; i < m; ++i){  //m条cbf约束 约束vx vy
        const Eigen::Vector2d diff = - nearest_obs[i];
        const double distance = diff.norm();
        const Eigen::Vector2d dir = (distance > kEps) ? diff / distance : Eigen::Vector2d(1.0,0.0); //距离极小的时候退化
        Atrip.emplace_back(row, 0,dir.x());
        Atrip.emplace_back(row, 1,dir.y());
        if(with_slack) Atrip.emplace_back(row, 3, 1.0);
        const double h = distance - p.d_safe;
        low(row) = -p.gamma * h;
        up(row) = kInf;
        ++row;
    }
    for(int i =0; i < 3; ++i){  //3条动力学约束 vx vy w
        Atrip.emplace_back(row, i, 1.0);
        low(row) = p.u_min(i);
        up(row) = p.u_max(i);
        row++;
    }
    if(with_slack){
        Atrip.emplace_back(row, 3,1.0);
        low(row) = 0.0;
        up(row) = kInf;
    }
    A.setFromTriplets(Atrip.begin(), Atrip.end());
    
    //调用osqp求解
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-4);
    solver.settings()->setRelativeTolerance(1e-4);
    solver.settings()->setPolish(true);

    solver.data()->setNumberOfVariables(nx); 
    solver.data()->setNumberOfConstraints(nr);
    if (!solver.data()->setHessianMatrix(P) || !solver.data()->setGradient(q) ||
      !solver.data()->setLinearConstraintsMatrix(A) ||
      !solver.data()->setLowerBound(low) || !solver.data()->setUpperBound(up)) {
      return false;
    }
    if (!solver.initSolver()) return false;
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
    if (solver.getStatus() != OsqpEigen::Status::Solved) return false;

    const Eigen::VectorXd x = solver.getSolution();
    u_out = x.head<3>();
    slack_out = with_slack ? x(3) : 0.0;
    return true;
}

}



CBFController::CBFController(const CBFParams& params): params_(params){}
CBFController::~CBFController() = default;

void CBFController::setParams(const CBFParams& params){ params_ = params;}
const CBFParams& CBFController::getParams() { return params_;};

CBFResult CBFController::filter(const Eigen::Vector3d& u_nom,
                 const std::vector<Eigen::Vector2d>& nearest_obs){
    CBFResult result;
    //先不做松弛
    if(solveQP(params_, u_nom, nearest_obs, result.u, false, result.slack_out)){
        result.solved = true;
        return result;
    }

    //失败后松弛
    if(params_.use_slack && solveQP(params_, u_nom, nearest_obs, result.u, true, result.slack_out)){
        result.solved = true;
        return result;
    }

    //都失败，返回 0速
    result.solved = false;
    result.u = {0.0,0.0,0.0};
    return result;
}

}