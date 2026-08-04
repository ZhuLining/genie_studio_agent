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


VALID_CODE_AND_MOTION_YAML = """
start_node: 开始
app_execution_id: code-and-motion-run
nodes:
  - id: 开始
    type: assign
    assignments: {}
  - id: 代码
    type: worker
    skill_name: script_skill
    params_template:
      script_id: code_echo_inputs
      input_mappings:
        - name: out_1
          type: string
          variable_ref: $.variables.system.detail.outputs.app_execution_id
      output_variables:
        - name: out_1
          type: string
      timeout: 50
    capture_state_detail: true
    output_var: 代码
    output_contract:
      required_paths:
        - $.variables.代码.detail
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
    to: 代码
  - from: 代码
    outcome: success
    to: 位姿调整-位控
  - from: 位姿调整-位控
    outcome: success
    to: 结束
"""


VALID_CODE_CHAIN_YAML = """
start_node: 开始
app_execution_id: code-chain-run
nodes:
  - id: 开始
    type: assign
    assignments: {}
  - id: 代码1
    type: worker
    skill_name: script_skill
    params_template:
      script_id: code_echo_inputs
      input_mappings:
        - name: out_1
          type: string
          variable_ref: $.variables.system.detail.outputs.app_execution_id
      output_variables:
        - name: out_1
          type: string
      timeout: 50
    capture_state_detail: true
    output_var: 代码1
    output_contract:
      required_paths:
        - $.variables.代码1.detail
  - id: 代码2
    type: worker
    skill_name: script_skill
    params_template:
      script_id: code_echo_inputs
      input_mappings:
        - name: out_2
          type: string
          variable_ref: $.variables.代码1.detail.outputs.out_1
      output_variables:
        - name: out_2
          type: string
      timeout: 50
    capture_state_detail: true
    output_var: 代码2
    output_contract:
      required_paths:
        - $.variables.代码2.detail
  - id: 结束
    type: assign
    assignments: {}
transitions:
  - from: 开始
    outcome: success
    to: 代码1
  - from: 代码1
    outcome: success
    to: 代码2
  - from: 代码2
    outcome: success
    to: 结束
"""


VALID_END_EFFECTOR_CODE_FLOW_YAML = """
start_node: 开始
app_execution_id: end-effector-code-flow
nodes:
  - id: 开始
    type: assign
    assignments: {}
  - id: 末端控制
    type: worker
    skill_name: control_end_effector_skill
    params_template:
      target_end: left_tool
      end_effector_type: omnipicker
      opening: 0.5
      timeout: 20
    capture_state_detail: true
    output_var: 末端控制
    output_contract:
      required_paths:
        - $.variables.末端控制.detail
  - id: 代码1
    type: worker
    skill_name: script_skill
    params_template:
      script_id: code_opening_plus_0p1
      input_mappings:
        - name: actual_openness
          type: array
          variable_ref: $.variables.末端控制.detail.outputs.actual_openness
      output_variables:
        - name: adjusted_opening
          type: number
      timeout: 50
    capture_state_detail: true
    output_var: 代码1
    output_contract:
      required_paths:
        - $.variables.代码1.detail
  - id: 代码2
    type: worker
    skill_name: script_skill
    params_template:
      script_id: code_move_end_effector
      input_mappings:
        - name: opening
          type: number
          variable_ref: $.variables.代码1.detail.outputs.adjusted_opening
        - name: target_end
          type: string
          variable_ref: $.variables.末端控制.detail.outputs.target_end
        - name: end_effector_type
          type: string
          variable_ref: $.variables.末端控制.detail.outputs.end_effector_type
      output_variables:
        - name: actual_openness
          type: array
      timeout: 50
    capture_state_detail: true
    output_var: 代码2
    output_contract:
      required_paths:
        - $.variables.代码2.detail
  - id: 结束
    type: assign
    assignments: {}
transitions:
  - from: 开始
    outcome: success
    to: 末端控制
  - from: 末端控制
    outcome: success
    to: 代码1
  - from: 代码1
    outcome: success
    to: 代码2
  - from: 代码2
    outcome: success
    to: 结束
"""


VALID_END_EFFECTOR_YAML = """
start_node: 开始
app_execution_id: end-effector-run
nodes:
  - id: 开始
    type: assign
    assignments: {}
  - id: 末端控制
    type: worker
    skill_name: control_end_effector_skill
    params_template:
      target_end: left_tool
      end_effector_type: omnipicker
      opening: 0.5
      timeout: 20
    capture_state_detail: true
    output_var: 末端控制
    output_contract:
      required_paths:
        - $.variables.末端控制.detail
  - id: 结束
    type: assign
    assignments: {}
transitions:
  - from: 开始
    outcome: success
    to: 末端控制
  - from: 末端控制
    outcome: success
    to: 结束
"""


VALID_LOOP_TIMER_YAML = """
start_node: 开始
app_execution_id: loop-timer-run
nodes:
  - id: 开始
    type: assign
    assignments: {}
  - id: 定时器
    type: timer
    timer_mode: rel
    duration: 0.2
  - id: 循环
    type: loop
    loop_mode: count
    children:
      - 代码1
      - 循环内定时器
      - 代码2
    iteration_max: 3
  - id: 代码1
    type: worker
    skill_name: script_skill
    params_template:
      script_id: code_echo_inputs
      input_mappings: []
      output_variables:
        - name: out_1
          type: string
      timeout: 50
    capture_state_detail: true
    output_var: 代码1
    output_contract:
      required_paths:
        - $.variables.代码1.detail
  - id: 循环内定时器
    type: timer
    timer_mode: rel
    duration: 0.1
  - id: 代码2
    type: worker
    skill_name: script_skill
    params_template:
      script_id: code_echo_inputs
      input_mappings:
        - name: out_2
          type: string
          variable_ref: $.variables.代码1.detail.outputs.out_1
      output_variables:
        - name: out_2
          type: string
      timeout: 50
    capture_state_detail: true
    output_var: 代码2
    output_contract:
      required_paths:
        - $.variables.代码2.detail
  - id: 结束
    type: assign
    assignments: {}
transitions:
  - from: 开始
    outcome: success
    to: 定时器
  - from: 定时器
    outcome: success
    to: 循环
  - from: 代码1
    outcome: success
    to: 循环内定时器
  - from: 循环内定时器
    outcome: success
    to: 代码2
  - from: 循环
    outcome: success
    to: 结束
"""
