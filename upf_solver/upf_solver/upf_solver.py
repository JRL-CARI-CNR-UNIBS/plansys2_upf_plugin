import rclpy
from rclpy.node import Node

from unified_planning.engines.results import PlanGenerationResultStatus
from unified_planning.environment import get_environment
from unified_planning.io import PDDLReader, PDDLWriter
from unified_planning.plot import plot_plan
from unified_planning.shortcuts import OneshotPlanner  # OptimalityGuarantee


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
        self.output_plan_path = (self.get_parameter('output_plan_path').get_parameter_value()
                                 ).string_value

        self.get_logger().info(f'Using solver: {self.solver}')
        self.get_logger().info(f'Using domain: {self.domain_path}')
        self.get_logger().info(f'Using problem: {self.problem_path}')
        self.get_logger().info(f'Output plan: {self.output_plan_path}')

    def check_solver(self):
        if self.solver not in get_environment().factory.engines:
            raise ValueError(f'Solver {self.solver} not found in available engines')

    def load_problem(self):
        reader = PDDLReader()
        try:
            self.parsed_problem = reader.parse_problem(self.domain_path, self.problem_path)
        except Exception as e:
            raise e

    def solve(self):
        with OneshotPlanner(name=self.solver) as planner:
            result = planner.solve(self.parsed_problem)
            self.get_logger().info(f'Plan generation result: {result.status}')
            if result.plan:
                self.get_logger().info(f'{result.plan}')

            writer = PDDLWriter(self.parsed_problem)
            with open(self.output_plan_path, 'w') as f:
                if (result.status == PlanGenerationResultStatus.SOLVED_SATISFICING or
                        result.status == PlanGenerationResultStatus.SOLVED_OPTIMALLY):
                    f.write(f'; Solution Found \n{writer.get_plan(result.plan)}')
                else:
                    f.write(f'; No solution {result.status}')
                    return
        plot_plan(result.plan, figsize=(18, 4))


def main(args=None):
    rclpy.init(args=args)
    node = UpfSolver()
    try:
        node.check_solver()
        node.load_problem()
        node.solve()
    except Exception as e:
        node.get_logger().error(f'Error! {e}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
