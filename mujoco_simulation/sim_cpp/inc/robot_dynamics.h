#pragma once
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>

class RobotDynamics 
{
public:
    RobotDynamics(const std::string& urdf_path, const std::string& mesh_dir, bool floating_base);

    void computeDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, Eigen::MatrixXd& M, Eigen::VectorXd& b);
    Eigen::VectorXd inverseDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::VectorXd& qdd);
    void printModelInfo();
    int getNQ() const { return model.nq; }
    int getNV() const { return model.nv; }

public:
    pinocchio::Model model;
    pinocchio::Data data;
};
