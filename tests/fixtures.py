VALID_RIGHT_ARM_YAML = """
start_node: 开始
app_execution_id: 977ddeb3-3a42-4027-9e6f-5a11bbb6ced9
nodes:
  - id: 开始
    type: assign
    assignments: {}
  - id: 位姿调整-位控
    type: worker
    skill_name: motion_plan_skill
    params_template:
      right_arm:
        control_type: ABS_JOINT
        action_data:
          - 0.282
          - -1.039
          - -0.304
          - -1.751
          - -0.621
          - -0.169
          - 1.122
      speed: 0.05
      timeout: 50
    capture_state_detail: true
    output_var: 位姿调整-位控
    output_contract:
      required_paths:
        - $.variables.位姿调整-位控.detail
  - id: 结束
    type: assign
    assignments: {}
transitions:
  - from: 开始
    outcome: success
    to: 位姿调整-位控
  - from: 位姿调整-位控
    outcome: success
    to: 结束
"""
