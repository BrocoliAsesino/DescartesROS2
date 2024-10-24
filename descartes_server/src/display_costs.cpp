// va a ser un coste de edges no de nodes es decir aue el punto 0 sera el cost de ir de 0 a 1, el 1 de 1 a 2 ....
#include <cmath>
#include <vector>
#include <Eigen/Dense> 
#include <fstream>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

class ViewCost  {

public:

    ViewCost(std::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> IkFastMoveitStateAdapterPtr, 
                                    Parameters& params) : IkFastMoveitStateAdapterPtr_(IkFastMoveitStateAdapterPtr),
                                    robot_state(IkFastMoveitStateAdapterPtr_->getRobotState()), margin_(params.margin),
                                    factorC1_(params.costFactorCSpaceDistance), factorC2_(params.costFactorDistanceLimits), 
                                    factorC3_(params.costFactorManipulability)
        {

        joint_model_group = IkFastMoveitStateAdapterPtr_->getJointModelGroup();
        group_name = joint_model_group->getName();
        
        
        }

    double CostCSpaceDistanceL1(const double* start, const double* stop);
    double CostManipulability(const double* stop, const std::string &joint_group_name, const std::string &tcp_frame);
    double CostCSpaceLogDistanceToLimits(const double* start);
    double computeForValues(double q, double qmin, double qmax);

    void iterateResults(trajectory_msgs::msg::JointTrajectory& joint_solutions){
        std::cout << "size of joint_solutions : " << joint_solutions.points.size() << std::endl;
        for (size_t i = 0; i < joint_solutions.points.size() - 1; ++i) {
        // std::cout << "iteration number: " << i << std::endl;
        if (i + 1 >= joint_solutions.points.size()) {
            std::cerr << "Error: Index out of bounds" << std::endl;
            break;
        }
        const auto& start_positions = joint_solutions.points[i].positions;
        const auto& stop_positions = joint_solutions.points[i + 1].positions;

        // std::cout << "Start positions: ";
        for (const auto& pos : start_positions) std::cout << pos << " ";
        std::cout << std::endl;
        // std::cout << "Stop positions: ";
        for (const auto& pos : stop_positions) std::cout << pos << " ";
        std::cout << std::endl;
        // Assuming start_positions and stop_positions are std::vector<double>
        calculateCosts(start_positions.data(), stop_positions.data());
        }

    }

    void calculateCosts(const double* start, const double* stop){
        // std::cout << "Calculating costs for start: " << start << " stop: " << stop << std::endl;

        double distCost = factorC1_ * CostCSpaceDistanceL1(start, stop);
        // std::cout << "CostCSpaceDistanceL1: " << distCost << std::endl;
        articulateDistCost.push_back(distCost);

        // double limitCost = factorC2_ * CostCSpaceLogDistanceToLimits(stop);
        // std::cout << "CostCSpaceLogDistanceToLimits: " << limitCost << std::endl;
        // articulateDistCost.push_back(limitCost);

        double manipulationCost = factorC3_ * CostManipulability(stop, group_name, "tcp");
        // std::cout << "CostManipulability: " << manipulationCost << std::endl;
        manipCost.push_back(manipulationCost);
    }

    void exportDataToFile(const std::string& filename) {
        std::ofstream file(filename);
        int iter_numb = 0;
        std::cout << articulateDistCost.size() << std::endl; 
        std::cout << manipCost.size() << std::endl;

        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }

        if (articulateDistCost.size() != manipCost.size()) {
            std::cerr << "Error: Vector sizes do not match" << std::endl;
            file.close();
            return;
        }

        for (size_t i = 0; i < manipCost.size(); ++i) {
            file << iter_numb << ", " << articulateDistCost[i] << ", " << manipCost[i] << std::endl;
            iter_numb++;
        }

        if (!file) {
            std::cerr << "Error occurred during file writing" << std::endl;
        }

        file.close();
    }
       

private: 

    std::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> IkFastMoveitStateAdapterPtr_;//It will be a pointer to the IkFastMoveitStateAdapter instance stored in the RobotModelPtr created in traj_planner
    moveit::core::RobotStatePtr robot_state;
    const moveit::core::JointModelGroup* joint_model_group;
    std::string group_name;
    std::vector<double> articulateDistCost, manipCost, limitsDistDistCost;
    double margin_;
    double factorC1_, factorC2_, factorC3_;
    

};

double  ViewCost::CostCSpaceDistanceL1(const double* start, const double* stop){
        double cost = 0;
        
        // for (int i = 0; i < IkFastMoveitStateAdapterPtr_->getDOF(); ++i)
        // {
        //     cost += std::abs(stop[i] - start[i]);
        // }

        // trying to penalize more the wrist movements -- to check
        for (int i = 0; i < IkFastMoveitStateAdapterPtr_->getDOF(); ++i)
        {
            double weight = 1.0 + 0.3*i; 
            cost += weight * std::abs(stop[i] - start[i]);
        }

        // std::cout << cost << std::endl;
        return cost;
}

double ViewCost::CostManipulability(const double* stop, const std::string &joint_group_name, const std::string &tcp_frame){

        moveit::core::RobotState startConfig_robot_state = *robot_state;

        // startConfig_robot_state.setVariablePositions(start);
        startConfig_robot_state.setJointGroupPositions(joint_group_name, stop);
        startConfig_robot_state.update();

        Eigen::MatrixXd jacobian;
        startConfig_robot_state.getJacobian(joint_model_group, robot_state->getLinkModel(tcp_frame), Eigen::Vector3d(0.0, 0.0, 0.0), jacobian);

        double det = (jacobian * jacobian.transpose()).determinant();
        // std::cout << "determinant value : " << det << std::endl;

        if (det <= 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("customCostFunction"), "Invalid Jacobian!");
            return std::numeric_limits<double>::infinity();
        }
        else if (det == 0){
            return std::numeric_limits<double>::infinity();
        }
        else
        {
            double m = std::sqrt(det);
            // double cost = 1.0/(1 + m);
            double cost = -log(m);
            return cost;
        }
}

double ViewCost::CostCSpaceLogDistanceToLimits(const double* start){
    double cost = 0;

    const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getJointModels();

    for(size_t i = 0; i < joint_models.size(); i++)
    {
        const auto& joint_bounds = joint_models[i]->getVariableBounds();
        cost += ViewCost::computeForValues(start[i], joint_bounds[i].min_position_, joint_bounds[i].max_position_);
    }

    return cost;
        
    }


double ViewCost::computeForValues(double q, double qmin, double qmax){

    if (this->margin_ <= 0.0)
        return std::numeric_limits<double>::infinity();
    double ret = 0.5*(
        std::max(0.0, -std::log((qmax-q)/this->margin_)) 
        + std::max(0.0, -std::log((q-qmin)/this->margin_))
    );
    return ret;
}


