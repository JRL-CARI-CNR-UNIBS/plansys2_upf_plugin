# Plansys2 UPF Plugin

This repository contains the plugin for [PlanSys2](https://github.com/PlanSys2/ros2_planning_system) that uses the [unified-planning (UP)](https://github.com/aiplan4eu/unified-planning) as a solver. There are two ROS 2 packages:

1. **upf_solver**: Contains a ROS 2 node that, given the domain and problem files in .pddl format and the path to save the plan, loads the domain/problem and saves the found solution.
2. **plansys2_upf_plan_solver**: Contains the actual plugin for PlanSys2.

## Installation

To install the plugin, follow these steps:

1. Clone the repository (in workspace src):

    ```sh
    git clone https://github.com/JRL-CARI-CNR-UNIBS/plansys2_upf_plugin.git
    ```
2. If you have not yet installed plansys2 (in workspace src):
    ```sh
    vcs import < plansys2_upf_plugin/dependencies.repos
    ```

3. Install dependencies using `rosdep` (in workspace level):

    ```sh
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Install Python dependencies (in workspace src):

    ```sh
    pip install -r plansys2_upf_plugin/upf_solver/requirements.txt
    ```

5. Build the packages (in workspace level):

    ```sh
    colcon build --symlink-install
    ```

## Packages

### upf_solver

The `upf_solver` package contains a ROS 2 node that performs the following tasks:
- **Inputs**:
  - Domain and problem files in .pddl format.
  - Path where the plan should be saved.
- **Functionality**:
  - Loads the domain and problem.
  - Computes the plan using UPF.
  - Saves the found solution to the specified path.
Example:

To use the `upf_solver` node, you need to start it with appropriate parameters. Example usage:

```sh
ros2 run upf_solver upf_solver --ros-args -p solver:='tamer' -p domain_path:='{$DOMAIN_PATH.PDDL}' -p problem_path:='{$PROBLEM_PATH.PDDL}' -p output_plan_path:='OUTPUT_PLAN.PDDL'
```

### plansys2_upf_plan_solver

The plansys2_upf_plan_solver package contains the PlanSys2 plugin that integrates the UPF solver. This plugin allows PlanSys2 to use UPF as a planning solver.

⚠️ When using the plugin for plansys first verify that the domain file and pddl problem used is formatted correctly (using ros2 run upf_solver ...). 

To configure PlanSys2 to use the UPF plugin, set the following parameters in your plansys2_bringup or planner node configuration file:


#### Configuration

To use this plugin in PlanSys2, set the following parameters in the `plansys2_bringup` or the planner node configuration:

```yaml
planner:
  ros__parameters:
    plan_solver_timeout: 15.0  # Adjust as needed
    plan_solver_plugins: ["UPF"]
    UPF:
      plugin: "plansys2/UPFPlanSolver"
      solver: "tamer" # tamer is by default (can be avoided the arg in that case), but there are others like "lpg"

```
