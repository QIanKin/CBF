#include <cmath>
#include <Eigen/Core>
#include <vector>

namespace controller {

//当前动力学模型为 {vx, vy, w}
struct CBFParams{
    double gamma;   // class-k函数 gamma * h(x)
    double d_safe;  // 需要的安全距离
    Eigen::Vector3d u_min{-1.0, -1.0, -1.5}; //动力学约束
    Eigen::Vector3d u_max{1.0, 1.0, 1.5};
    Eigen::Vector3d w_cost{1.0,1.0,1.0};  //代价函数权重
    double w_slack = 1e3;   //松弛变量代价权重
    bool use_slack = true;  
};

struct CBFResult{
    Eigen::Vector3d u{0.0, 0.0, 0.0};
    double slack_out = 0.0;
    bool solved = false;
};

class CBFController{

public:
    explicit CBFController(const CBFParams& params);
    ~CBFController();

    void setParams(const CBFParams& params);
    const CBFParams& getParams();

    CBFResult filter(const Eigen::Vector3d& u_nom,
                     const std::vector<Eigen::Vector2d>& nearest_obs);
private:
    CBFParams params_;
};

}