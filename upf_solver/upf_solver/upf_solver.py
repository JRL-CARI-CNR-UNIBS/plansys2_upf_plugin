import rclpy
from rclpy.node import Node

from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.shortcuts import OneshotPlanner # OptimalityGuarantee


class UpfSolver(Node):
    def __init__(self):
        super().__init__('upf_solver_node')

        self.declare_parameter('solver', '')
        self.declare_parameter('domain_path', '')
        self.declare_parameter('problem_path', '')
        self.declare_parameter('output_plan_path', '')

        self.solver = self.get_parameter('solver').get_parameter_value().string_value
        self.domain_path = self.get_parameter('domain_path').get_parameter_value().string_value
        self.problem_path = self.get_parameter('problem_path').get_parameter_value().string_value
        self.output_plan_path = self.get_parameter('output_plan_path').get_parameter_value().string_value

        self.get_logger().info(f'Using solver: {self.solver}')
        self.get_logger().info(f'Using domain: {self.domain_path}')
        self.get_logger().info(f'Using problem: {self.problem_path}')
        self.get_logger().info(f'Output plan: {self.output_plan_path}')

    def load_problem(self):
        reader = PDDLReader()
        try:
            self.parsed_problem = reader.parse_problem(self.domain_path, self.problem_path)
        except Exception as e:
            self.get_logger().error(f'Error loading problem: {e}')

    def solve(self):
        with OneshotPlanner(name=self.solver) as planner: # optimality_guarantee=OptimalityGuarantee.SOLVED_OPTIMALLY
            result = planner.solve(self.parsed_problem)
            print(result.status)
            print(result.plan)
            writer = PDDLWriter(self.parsed_problem)
            writer.write_plan(result.plan, self.output_plan_path)

def main(args=None):
    rclpy.init(args=args)
    node = UpfSolver()
    try:
        node.load_problem()
        node.solve()
    except Exception as e:
        node.get_logger().error(f'Error solving problem: {e}')  
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
