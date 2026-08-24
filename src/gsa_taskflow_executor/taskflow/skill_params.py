"""Taskflow worker 节点的技能参数模型解析。

parser 只负责 DAG 结构；技能参数在这里按 skill_name 分发校验。这样后续新增
条件判断/并行等结构节点时，不会继续把 GDK/脚本参数细节堆进 YAML parser。
"""

from __future__ import annotations

from typing import Any

from gsa_taskflow_executor.code_scripts.registry import CODE_SCRIPT_IDS
from gsa_taskflow_executor.taskflow.models import (
    EndEffectorParams,
    ForceControlParams,
    LoopParams,
    MotionPlanParams,
    MotionPlanTarget,
    QrPoseParams,
    ScriptInputMapping,
    ScriptOutputVariable,
    ScriptParams,
    TaskflowParseError,
    TimerParams,
    YamlMapping,
)
from gsa_taskflow_executor.taskflow.readers import (
    expect_mapping,
    read_bool,
    read_int_in_range_with_default,
    read_non_negative_number,
    read_non_negative_number_with_default,
    read_number_in_range_with_default,
    read_optional_string,
    read_positive_int,
    read_positive_number,
    read_positive_number_with_default,
    read_required_string,
    read_required_string_list,
    to_float,
)

# 运动速度限制（GDK velocity 单位）
MOTION_SPEED_MIN = 0.001
MOTION_SPEED_MAX = 0.1

# 默认超时（秒）
DEFAULT_SCRIPT_TIMEOUT = 50.0
DEFAULT_END_EFFECTOR_TIMEOUT = 20.0
DEFAULT_FORCE_CONTROL_TIMEOUT = 50.0
DEFAULT_QR_POSE_TIMEOUT = 60.0
DEFAULT_QR_POSE_MIN_MARKERS = 3
DEFAULT_QR_POSE_RETURN_TO_INITIAL_PHOTO_POSE = True
DEFAULT_QR_POSE_RETURN_POSE_SPEED = 0.03
DEFAULT_QR_POSE_RETURN_POSE_TIMEOUT = 50.0
MAX_QR_POSE_MIN_MARKERS = 32

# 末端执行器默认值
DEFAULT_END_EFFECTOR_POST_WAIT_SECONDS = 1.0  # 动作完成后的稳定等待

# 力控默认参数
DEFAULT_FORCE_CONTROL_HZ = 50
DEFAULT_FORCE_CONTROL_STEP = 0.01

# 循环控制
LOOP_MODE_COUNT = "count"  # v1 唯一支持的循环模式
MAX_LOOP_ITERATIONS = 100

# 定时器
TIMER_MODE_REL = "rel"  # 相对定时器

# 力控方法
FORCE_CONTROL_METHOD_SMOOTH_MOVE = "smooth_move"
FORCE_CONTROL_METHOD_MOVE_UNTIL_FORCE = "move_until_force"
FORCE_CONTROL_METHODS = {
    FORCE_CONTROL_METHOD_SMOOTH_MOVE,
    FORCE_CONTROL_METHOD_MOVE_UNTIL_FORCE,
}
FORCE_CONTROL_ARMS = {"left_arm", "right_arm"}
FORCE_CONTROL_HZ_MIN = 1
FORCE_CONTROL_HZ_MAX = 100
FORCE_CONTROL_STEP_MIN = 0.001
FORCE_CONTROL_STEP_MAX = 0.05

# 末端执行器目标名称别名 → 规范名
END_EFFECTOR_TARGET_ALIASES = {
    "left": "left_tool",
    "left_end": "left_tool",
    "left_tool": "left_tool",
    "左末端": "left_tool",
    "right": "right_tool",
    "right_end": "right_tool",
    "right_tool": "right_tool",
    "右末端": "right_tool",
    "dual": "dual_tool",
    "both": "dual_tool",
    "dual_tool": "dual_tool",
    "双末端": "dual_tool",
}

SCRIPT_IDS = CODE_SCRIPT_IDS

# 脚本值类型别名归一化（如 "String" → "string"）
SCRIPT_VALUE_TYPE_ALIASES = {
    "string": "string",
    "String": "string",
    "integer": "integer",
    "Integer": "integer",
    "number": "number",
    "Number": "number",
    "boolean": "boolean",
    "Boolean": "boolean",
    "time": "time",
    "Time": "time",
    "object": "object",
    "Object": "object",
    "array": "array",
    "Array": "array",
}


def validate_worker_params(skill_name: str, params: YamlMapping, path: str) -> None:
    """按 skill_name 分发校验 params_template。"""

    if skill_name == "motion_plan_skill":
        parse_motion_plan_params(params, f"{path}.params_template")
    elif skill_name == "script_skill":
        parse_script_params(params, f"{path}.params_template")
    elif skill_name == "control_end_effector_skill":
        parse_end_effector_params(params, f"{path}.params_template")
    elif skill_name == "force_control_skill":
        parse_force_control_params(params, f"{path}.params_template")
    elif skill_name == "qr_pose_skill":
        parse_qr_pose_params(params, f"{path}.params_template")


def parse_timer_params(params: YamlMapping, path: str) -> TimerParams:
    """解析 timer 节点参数。"""

    timer_mode = read_required_string(params, "timer_mode", path)
    if timer_mode != TIMER_MODE_REL:
        raise TaskflowParseError(f"{path}.timer_mode 目前只支持 {TIMER_MODE_REL}")
    return TimerParams(
        timer_mode=timer_mode,
        duration=read_non_negative_number(params.get("duration"), f"{path}.duration"),
    )


def parse_loop_params(params: YamlMapping, path: str) -> LoopParams:
    """解析 loop 节点参数。"""

    loop_mode = read_required_string(params, "loop_mode", path)
    if loop_mode != LOOP_MODE_COUNT:
        raise TaskflowParseError(f"{path}.loop_mode 目前只支持 {LOOP_MODE_COUNT}")
    return LoopParams(
        loop_mode=loop_mode,
        children=tuple(read_required_string_list(params.get("children"), f"{path}.children")),
        iteration_max=read_positive_int(
            params.get("iteration_max"),
            f"{path}.iteration_max",
            maximum=MAX_LOOP_ITERATIONS,
        ),
    )


def parse_motion_plan_params(params: YamlMapping, path: str) -> MotionPlanParams:
    """解析 motion_plan_skill 参数。"""

    speed = read_motion_speed(params.get("speed"), f"{path}.speed")
    timeout = read_positive_number(params.get("timeout"), f"{path}.timeout")
    targets: list[MotionPlanTarget] = []

    for body_part in ("left_arm", "right_arm", "waist"):
        if body_part not in params:
            continue
        target = expect_mapping(params[body_part], f"{path}.{body_part}")
        control_type = read_required_string(target, "control_type", f"{path}.{body_part}")
        action_data = parse_action_data(
            target.get("action_data"),
            body_part,
            control_type,
            f"{path}.{body_part}.action_data",
        )
        targets.append(
            MotionPlanTarget(
                body_part=body_part,
                control_type=control_type,
                action_data=action_data,
            )
        )

    if not targets:
        raise TaskflowParseError(f"{path} 至少需要 left_arm/right_arm/waist 之一")

    return MotionPlanParams(targets=tuple(targets), speed=speed, timeout=timeout)


def parse_qr_pose_params(params: YamlMapping, path: str) -> QrPoseParams:
    """解析二维码定位节点参数。

    该 skill 会先按初始拍照点位回到手臂和腰部采样姿态，再重新拍照定位二维码，
    输出目标点 pose/action_data。回位动作仍复用 ABS_JOINT 安全门。
    """

    arm = read_required_string(params, "arm", path)
    if arm not in {"left_arm", "right_arm"}:
        raise TaskflowParseError(f"{path}.arm 必须是 left_arm 或 right_arm")
    camera_id = read_required_string(params, "camera_id", path)
    expected_camera = "hand_left_color" if arm == "left_arm" else "hand_right_color"
    if camera_id != expected_camera:
        raise TaskflowParseError(f"{path}.camera_id 与 {arm} 不匹配，应为 {expected_camera}")
    return QrPoseParams(
        robot_serial=read_required_string(params, "robot_serial", path),
        project_name=read_required_string(params, "project_name", path),
        initial_photo_point_name=read_required_string(
            params,
            "initial_photo_point_name",
            path,
        ),
        map_name=read_optional_string(params.get("map_name"), f"{path}.map_name"),
        arm=arm,
        camera_id=camera_id,
        timeout=read_positive_number_with_default(
            params.get("timeout"),
            f"{path}.timeout",
            DEFAULT_QR_POSE_TIMEOUT,
        ),
        min_markers=read_positive_int(
            params.get("min_markers"),
            f"{path}.min_markers",
            maximum=MAX_QR_POSE_MIN_MARKERS,
        )
        if params.get("min_markers") is not None
        else DEFAULT_QR_POSE_MIN_MARKERS,
        return_to_initial_photo_pose=read_bool(
            params.get("return_to_initial_photo_pose"),
            DEFAULT_QR_POSE_RETURN_TO_INITIAL_PHOTO_POSE,
        ),
        return_pose_speed=read_number_in_range_with_default(
            params.get("return_pose_speed"),
            f"{path}.return_pose_speed",
            fallback=DEFAULT_QR_POSE_RETURN_POSE_SPEED,
            minimum=MOTION_SPEED_MIN,
            maximum=MOTION_SPEED_MAX,
        ),
        return_pose_timeout=read_positive_number_with_default(
            params.get("return_pose_timeout"),
            f"{path}.return_pose_timeout",
            DEFAULT_QR_POSE_RETURN_POSE_TIMEOUT,
        ),
    )


def parse_script_params(params: YamlMapping, path: str) -> ScriptParams:
    """解析 script_skill 参数。校验 script_id 白名单。"""

    script_id = read_required_string(params, "script_id", path)
    if script_id not in SCRIPT_IDS:
        raise TaskflowParseError(f"{path}.script_id 未在白名单中: {script_id}")
    timeout = read_positive_number_with_default(
        params.get("timeout"),
        f"{path}.timeout",
        DEFAULT_SCRIPT_TIMEOUT,
    )
    input_mappings = parse_script_input_mappings(
        params.get("input_mappings", ()),
        f"{path}.input_mappings",
    )
    output_variables = parse_script_output_variables(
        params.get("output_variables", ()),
        f"{path}.output_variables",
    )
    return ScriptParams(
        script_id=script_id,
        timeout=timeout,
        input_mappings=tuple(input_mappings),
        output_variables=tuple(output_variables),
    )


def parse_script_input_mappings(raw: Any, path: str) -> list[ScriptInputMapping]:
    """解析脚本输入映射。空行自动跳过。"""

    if raw in (None, ()):
        return []
    if not isinstance(raw, list):
        raise TaskflowParseError(f"{path} 必须是数组")

    seen_names: set[str] = set()
    mappings: list[ScriptInputMapping] = []
    for index, item in enumerate(raw):
        item_path = f"{path}[{index}]"
        mapping = expect_mapping(item, item_path)
        if is_blank_script_mapping_row(mapping, ("name", "type", "variable_ref", "variableRef")):
            continue
        name = read_required_string(mapping, "name", item_path)
        validate_script_variable_name(name, f"{item_path}.name")
        if name in seen_names:
            raise TaskflowParseError(f"{item_path}.name 重复: {name}")
        seen_names.add(name)
        variable_ref = read_script_variable_ref(mapping, item_path)
        mappings.append(
            ScriptInputMapping(
                name=name,
                value_type=read_script_value_type(mapping.get("type"), f"{item_path}.type"),
                variable_ref=variable_ref,
            )
        )
    return mappings


def parse_script_output_variables(raw: Any, path: str) -> list[ScriptOutputVariable]:
    """解析脚本输出变量声明。空行自动跳过。"""

    if raw in (None, ()):
        return []
    if not isinstance(raw, list):
        raise TaskflowParseError(f"{path} 必须是数组")

    seen_names: set[str] = set()
    outputs: list[ScriptOutputVariable] = []
    for index, item in enumerate(raw):
        item_path = f"{path}[{index}]"
        output = expect_mapping(item, item_path)
        if is_blank_script_mapping_row(output, ("name", "type")):
            continue
        name = read_required_string(output, "name", item_path)
        validate_script_variable_name(name, f"{item_path}.name")
        if name in seen_names:
            raise TaskflowParseError(f"{item_path}.name 重复: {name}")
        seen_names.add(name)
        outputs.append(
            ScriptOutputVariable(
                name=name,
                value_type=read_script_value_type(output.get("type"), f"{item_path}.type"),
            )
        )
    return outputs


def parse_end_effector_params(params: YamlMapping, path: str) -> EndEffectorParams:
    """解析 control_end_effector_skill 参数。target_end 支持中文别名。"""

    target_end = read_end_effector_target(params, path)
    end_effector_type = read_optional_string(
        params.get("end_effector_type", params.get("target_type")),
        f"{path}.end_effector_type",
    )
    left_end_effector_type = read_optional_string(
        params.get("left_end_effector_type", params.get("left_target_type")),
        f"{path}.left_end_effector_type",
    )
    right_end_effector_type = read_optional_string(
        params.get("right_end_effector_type", params.get("right_target_type")),
        f"{path}.right_end_effector_type",
    )
    opening = read_end_effector_opening(
        params.get("opening"),
        f"{path}.opening",
        required=False,
    )
    left_opening = read_end_effector_opening(
        params.get("left_opening"),
        f"{path}.left_opening",
        required=False,
    )
    right_opening = read_end_effector_opening(
        params.get("right_opening"),
        f"{path}.right_opening",
        required=False,
    )
    validate_end_effector_opening_presence(
        target_end=target_end,
        opening=opening,
        left_opening=left_opening,
        right_opening=right_opening,
        path=path,
    )
    timeout = read_positive_number_with_default(
        params.get("timeout"),
        f"{path}.timeout",
        DEFAULT_END_EFFECTOR_TIMEOUT,
    )
    post_wait_seconds = read_non_negative_number_with_default(
        params.get("post_wait_seconds", params.get("postWaitSeconds")),
        f"{path}.post_wait_seconds",
        DEFAULT_END_EFFECTOR_POST_WAIT_SECONDS,
    )
    return EndEffectorParams(
        target_end=target_end,
        end_effector_type=end_effector_type,
        opening=opening,
        timeout=timeout,
        post_wait_seconds=post_wait_seconds,
        left_end_effector_type=left_end_effector_type,
        right_end_effector_type=right_end_effector_type,
        left_opening=left_opening,
        right_opening=right_opening,
    )


def validate_end_effector_opening_presence(
    *,
    target_end: str,
    opening: float | None,
    left_opening: float | None,
    right_opening: float | None,
    path: str,
) -> None:
    """真机末端控制必须带显式开度，避免 malformed YAML 被默认值驱动动作。"""

    if target_end == "left_tool" and opening is None and left_opening is None:
        raise TaskflowParseError(f"{path}.opening 或 {path}.left_opening 必须提供")
    if target_end == "right_tool" and opening is None and right_opening is None:
        raise TaskflowParseError(f"{path}.opening 或 {path}.right_opening 必须提供")
    if target_end == "dual_tool" and opening is None:
        if left_opening is None or right_opening is None:
            raise TaskflowParseError(
                f"{path}.dual_tool 必须提供 opening，或同时提供 left_opening/right_opening"
            )


def parse_force_control_params(params: YamlMapping, path: str) -> ForceControlParams:
    """解析 force_control_skill 参数。线上硬阻断但保留校验。"""

    method = read_required_string(params, "method", path)
    if method not in FORCE_CONTROL_METHODS:
        raise TaskflowParseError(f"{path}.method 只支持 smooth_move/move_until_force")

    arm = read_required_string(params, "arm", path)
    if arm not in FORCE_CONTROL_ARMS:
        raise TaskflowParseError(f"{path}.arm 只支持 left_arm/right_arm")

    force_threshold: float | None = None
    if method == FORCE_CONTROL_METHOD_MOVE_UNTIL_FORCE:
        force_threshold = read_positive_number(
            params.get("force_threshold"),
            f"{path}.force_threshold",
        )

    return ForceControlParams(
        method=method,
        arm=arm,
        delta_xyz=read_force_delta_xyz(params.get("delta_xyz"), f"{path}.delta_xyz"),
        force_threshold=force_threshold,
        timeout_s=read_positive_number_with_default(
            params.get("timeout_s"),
            f"{path}.timeout_s",
            DEFAULT_FORCE_CONTROL_TIMEOUT,
        ),
        control_hz=read_int_in_range_with_default(
            params.get("control_hz"),
            f"{path}.control_hz",
            fallback=DEFAULT_FORCE_CONTROL_HZ,
            minimum=FORCE_CONTROL_HZ_MIN,
            maximum=FORCE_CONTROL_HZ_MAX,
        ),
        step=read_number_in_range_with_default(
            params.get("step"),
            f"{path}.step",
            fallback=DEFAULT_FORCE_CONTROL_STEP,
            minimum=FORCE_CONTROL_STEP_MIN,
            maximum=FORCE_CONTROL_STEP_MAX,
        ),
    )


def read_force_delta_xyz(value: Any, path: str) -> tuple[float, float, float]:
    """解析并校验力控 delta_xyz 位移向量（禁止零向量）。"""

    if not isinstance(value, list):
        raise TaskflowParseError(f"{path} 必须是长度为 3 的数字数组")
    if len(value) != 3:
        raise TaskflowParseError(f"{path} 长度必须是 3，当前为 {len(value)}")

    x = to_float(value[0], f"{path}[0]")
    y = to_float(value[1], f"{path}[1]")
    z = to_float(value[2], f"{path}[2]")
    values = (x, y, z)
    if all(abs(item) < 1e-12 for item in values):
        raise TaskflowParseError(f"{path} 不能全为 0")
    return values


def parse_action_data(raw: Any, body_part: str, control_type: str, path: str) -> list[float] | str:
    """解析运动 action_data。支持 $.variables.* 引用字符串或关节角度数组。

    arm 期望 7 关节，waist 期望 5 关节。
    """

    if isinstance(raw, str):
        value = raw.strip()
        if value.startswith("$.variables."):
            return value

    if control_type not in {"ABS_JOINT", "ABS_POSE", "DELTA_POSE"}:
        raise TaskflowParseError(f"{path}.control_type 不支持: {control_type}")

    if not isinstance(raw, list):
        raise TaskflowParseError(f"{path} 必须是数值数组")

    if body_part == "waist" and control_type != "ABS_JOINT":
        raise TaskflowParseError(f"{path} 腰部第一阶段只支持 ABS_JOINT")
    expected_length = 5 if body_part == "waist" and control_type == "ABS_JOINT" else 7
    if len(raw) != expected_length:
        raise TaskflowParseError(
            f"{path} 长度必须是 {expected_length}，当前为 {len(raw)}"
        )

    values = [to_float(item, f"{path}[{index}]") for index, item in enumerate(raw)]
    return values


def read_script_variable_ref(mapping: YamlMapping, path: str) -> str:
    """读取 variable_ref/variableRef 字段，必须以 $.variables. 开头。"""

    raw = mapping.get("variable_ref", mapping.get("variableRef"))
    if not isinstance(raw, str) or not raw.strip():
        raise TaskflowParseError(f"{path}.variable_ref 必须是非空字符串")
    variable_ref = raw.strip()
    if not variable_ref.startswith("$.variables."):
        raise TaskflowParseError(f"{path}.variable_ref 必须是 $.variables. 开头的变量引用")
    return variable_ref


def read_script_value_type(value: Any, path: str) -> str:
    """归一化脚本值类型（如 "String" → "string"）。"""

    if not isinstance(value, str) or not value.strip():
        raise TaskflowParseError(f"{path} 必须是非空字符串")
    value_type = SCRIPT_VALUE_TYPE_ALIASES.get(value.strip())
    if value_type is None:
        allowed = "/".join(sorted(set(SCRIPT_VALUE_TYPE_ALIASES.values())))
        raise TaskflowParseError(f"{path} 类型无效，只支持 {allowed}")
    return value_type


def validate_script_variable_name(name: str, path: str) -> None:
    """变量名禁止含 "."（避免在 $.variables.<node>.<path> 中解析到错误层级）。"""

    if "." in name:
        raise TaskflowParseError(f"{path} 不能包含 .")


def is_blank_script_mapping_row(mapping: YamlMapping, keys: tuple[str, ...]) -> bool:
    """检查映射行是否所有指定字段均为空（用于跳过多余空行）。"""

    for key in keys:
        value = mapping.get(key)
        if isinstance(value, str) and value.strip():
            return False
        if value is not None and not isinstance(value, str):
            return False
    return True


def read_end_effector_target(params: YamlMapping, path: str) -> str:
    """解析 target_end 别名（如 left → left_tool，双末端 → dual_tool）。"""

    raw = read_required_string(params, "target_end", path)
    target = END_EFFECTOR_TARGET_ALIASES.get(raw)
    if target is None:
        raise TaskflowParseError(f"{path}.target_end 只支持 left_tool/right_tool/dual_tool")
    return target


def read_end_effector_opening(value: Any, path: str, *, required: bool = True) -> float | None:
    """读取末端开度值，范围 [0, 1]。0 = 闭合，1 = 全开。"""

    if value is None and not required:
        return None
    number = to_float(value, path)
    if number < 0 or number > 1:
        raise TaskflowParseError(f"{path} 必须在 0 到 1 之间")
    return number


def read_motion_speed(value: Any, path: str) -> float:
    """读取运动速度，范围 [MOTION_SPEED_MIN, MOTION_SPEED_MAX]。"""

    number = read_positive_number(value, path)
    if number < MOTION_SPEED_MIN or number > MOTION_SPEED_MAX:
        raise TaskflowParseError(
            f"{path} 必须在 {MOTION_SPEED_MIN} 到 {MOTION_SPEED_MAX} 之间"
        )
    return number
