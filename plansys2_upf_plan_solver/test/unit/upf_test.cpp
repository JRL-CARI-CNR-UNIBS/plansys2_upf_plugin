// Copyright 2024 National Research Council - Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <fstream>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "plansys2_upf_plan_solver/upf_plan_solver.hpp"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "plansys2_core/PlanSolverBase.hpp"


// TO TEST GUIDELINES:
// 1. Test the UPFPlanSolver::getPlan method with a valid domain and problem.
// 2. Test the UPFPlanSolver::getPlan method with an invalid domain and problem.
// 3. Test the UPFPlanSolver::isDomainValid method with a not existing solver
// 4. Test the UPFPlanSolver::isDomainValid with wrong domain and problem path

class UPFPlanSolverTest : public ::testing::Test
{
public:
  UPFPlanSolverTest()
  {
    std::cerr << "UPFPlanSolverTest" << std::endl;
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared("test_node");
    planner_ = std::make_shared<plansys2::UPFPlanSolver>();
    configure_planner("tamer");
  }
  ~UPFPlanSolverTest()
  {
    node_.reset();
    planner_.reset();
  }
  void configure_planner(const std::string & solver)
  {
    planner_->configure(node_, "UPF");  // Internally is declared UPF.solver
    node_->set_parameter(rclcpp::Parameter("UPF.solver", solver));
  }
  void load_domain_problem(
    const std::string & domain_name,
    const std::string & problem_name)
  {
    std::string pkgpath = ament_index_cpp::get_package_share_directory("plansys2_upf_plan_solver");

    std::ifstream domain_ifs(pkgpath + "/pddl/" + domain_name);
    domain_.assign(
      (std::istreambuf_iterator<char>(domain_ifs)),
      std::istreambuf_iterator<char>());

    std::ifstream problem_ifs(pkgpath + "/pddl/" + problem_name);
    problem_.assign(
      (std::istreambuf_iterator<char>(problem_ifs)),
      std::istreambuf_iterator<char>());
  }
  std::optional<plansys2_msgs::msg::Plan> solve()
  {
    std::cerr << "solve" << std::endl;
    return planner_->getPlan(domain_, problem_, "generate_plan_good");
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string domain_;
  std::string problem_;
  std::shared_ptr<plansys2::UPFPlanSolver> planner_;
};

TEST_F(UPFPlanSolverTest, GeneratePlan)
{
  load_domain_problem("domain.pddl", "problem.pddl");
  auto plan = solve();
  for (const auto & item : plan->items) {
    std::cerr << item.action.c_str() << std::endl;
  }
  ASSERT_TRUE(plan);
  ASSERT_EQ(plan.value().items.size(), 10);
}

TEST_F(UPFPlanSolverTest, GenerateInvalidPlan)
{
  load_domain_problem("no_solution_domain.pddl", "no_solution_problem.pddl");
  auto plan = solve();

  ASSERT_FALSE(plan.has_value());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
