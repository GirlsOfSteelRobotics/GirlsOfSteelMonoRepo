"""
Runs a regex replace on any of the class names that got changed in the past year updates of wpilib / vendor deps
"""
import json
from libraries.scripts.updater.utils import (
    PINNED_VSCODE_WPILIB_COMMITISH,
    DEFAULT_DIR_BLACKLIST,
    walk_for_extension,
    regex_replace_file,
    auto_retry_download,
)
from libraries.scripts.git.git_python_wrappers import commit_all_changes


def __run_replacement(replacements, root=".", dir_blacklist=None):
    java_files = walk_for_extension(root, "java", dir_blacklist=dir_blacklist)

    for java_file in java_files:
        regex_replace_file(java_file, replacements)


def run_standard_replacement(auto_commit):
    # Last sync Dec 19, 2021
    wpilib_replacements_url = f"https://raw.githubusercontent.com/wpilibsuite/vscode-wpilib/{PINNED_VSCODE_WPILIB_COMMITISH}/vscode-wpilib/resources/java_replacements.json"

    raw_json_data = auto_retry_download(wpilib_replacements_url).decode("utf-8")
    json_data = json.loads(raw_json_data)

    replacements = []
    for replacement_json in json_data[0]["replacements"]:
        replacement_to = replacement_json["to"]
        # Python-ize the replacement substitution
        replacement_to = replacement_to.replace("$1", r"\1").replace("$2", r"\2")
        if "$" in replacement_to:
            raise Exception(f"Make this smarter. To = '{replacement_to}")
        replacements.append((replacement_json["from"], replacement_to))

    # Run these on all the files
    __run_replacement(replacements)

    if auto_commit:
        commit_all_changes("Auto-Update: Ran standard vscode replacements")


def run_our_additional_replacements(auto_commit):
    replacements = []

    # Put our smarter-than-wpilib replacements here
    # fmt: off
    replacements.append(("GosCommand", "GosCommandBase"))
    replacements.append(("GosCommandBaseBase", "GosCommandBase"))
    # fmt: on

    # Run these on all the files
    __run_replacement(replacements)

    if auto_commit:
        commit_all_changes("Auto-Update: Ran our additional replacements")


def run_rev_replacements(auto_commit):
    replacements = []

    # fmt: off
    replacements.append(("import com.revrobotics.CANSparkLowLevel;", "import com.revrobotics.spark.SparkLowLevel;"))
    replacements.append(("import com.revrobotics.CANSparkMax;", "import com.revrobotics.spark.SparkMax;"))
    replacements.append(("import com.revrobotics.SparkPIDController;", "import com.revrobotics.spark.SparkClosedLoopController;"))
    replacements.append(("import com.revrobotics.CANSparkBase;", "import com.revrobotics.spark.SparkBase;"))
    replacements.append(("import com.revrobotics.SimableCANSparkFlex;", "import com.revrobotics.spark.SparkFlex;"))
    replacements.append(("import com.revrobotics.SimableCANSparkMax;", "import com.revrobotics.spark.SparkMax;"))
    replacements.append(("import com.revrobotics.SparkClosedLoopController.ArbFFUnits;", "import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;"))

    replacements.append(("import com.revrobotics.CANSparkBase.ControlType;", "import com.revrobotics.spark.SparkBase.ControlType;"))
    replacements.append(("import com.revrobotics.CANSparkBase.IdleMode;", "import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;"))
    replacements.append(("import com.revrobotics.CANSparkLowLevel.MotorType;", "import com.revrobotics.spark.SparkLowLevel.MotorType;"))
    replacements.append(("import com.revrobotics.SparkAbsoluteEncoder;", "import com.revrobotics.spark.SparkAbsoluteEncoder;"))
    replacements.append(("import com.revrobotics.CANSparkLowLevel.PeriodicFrame;", ""))

    replacements.append(("SimableCANSparkMax", "SparkMax"))
    replacements.append(("SimableCANSparkFlex", "SparkFlex"))
    replacements.append(("CANSparkBase", "SparkBase"))
    replacements.append(("SparkPIDController", "SparkClosedLoopController"))
    replacements.append(("getPIDController", "getClosedLoopController"))

    replacements.append(("AbsEncoder.setInverted", ".absoluteEncoder.inverted"))
    replacements.append(("AbsEncoder.setZeroOffset", ".absoluteEncoder.zeroOffset"))

    replacements.append(("m_(.*).setPeriodicFramePeriod\(PeriodicFrame.kStatus5, ", r"\1Config.signals.absoluteEncoderPositionPeriodMs("))
    replacements.append(("m_(.*).setPeriodicFramePeriod\(PeriodicFrame.kStatus6, ", r"\1Config.signals.absoluteEncoderVelocityPeriodMs("))
    replacements.append(("m_(.*).setPeriodicFramePeriod\(CANSparkLowLevel.PeriodicFrame.kStatus5, ", r"\1Config.signals.absoluteEncoderPositionPeriodMs("))
    replacements.append(("m_(.*).setPeriodicFramePeriod\(CANSparkLowLevel.PeriodicFrame.kStatus6, ", r"\1Config.signals.absoluteEncoderVelocityPeriodMs("))

    replacements.append(("m_(.*).setIdleMode\(IdleMode", r"\1Config.idleMode(IdleMode"))
    replacements.append(("m_(.*).setIdleMode\(idleMode", r"\1Config.idleMode(idleMode"))
    replacements.append(("m_(.*).setSmartCurrentLimit", r"\1Config.smartCurrentLimit"))
    replacements.append(("m_(.*).setPositionConversionFactor", r"\1Config._encoder_.positionConversionFactor"))
    replacements.append(("m_(.*).setVelocityConversionFactor", r"\1Config._encoder_.velocityConversionFactor"))
    replacements.append(("m_(.*).enableVoltageCompensation", r"\1Config.voltageCompensation"))

    replacements.append(("m_(.*).restoreFactoryDefaults", r"SparkMaxConfig \1Config = new SparkMaxConfig"))
    replacements.append(("m_(.*).burnFlash\(\)", r"m_\1.configure(\1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)"))

    replacements.append(("m_(.*).follow\(", r"\1Config.follow("))

    replacements.append(("setPositionPIDWrappingEnabled", "closedLoop.positionWrappingEnabled"))
    replacements.append(("setPositionPIDWrappingMinInput", "closedLoop.positionWrappingMinInput"))
    replacements.append(("setPositionPIDWrappingMaxInput", "closedLoop.positionWrappingMaxInput"))

    replacements.append(("import com.revrobotics.spark.SparkMax;", """import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;"""))
    # fmt: on

    # Run these on all the files
    __run_replacement(replacements, dir_blacklist=DEFAULT_DIR_BLACKLIST + [
        "ChassisProject",
        "y2016",
        "y2017",
        "y2018",
        "y2019",
    ])

    if auto_commit:
        commit_all_changes("Auto-Update: Ran rev replacements")


def snowflake_rev_replacements(auto_commit):
    replacements_map = {}
    replacements_map["libraries/GirlsOfSteelLibRev/src/main/java/com/gos/lib/rev/swerve/RevSwerveModule.java"] = [
        ("// SparkMaxConfig drivingSparkMaxConfig", "SparkMaxConfig drivingMotorConfig"),
        ("// SparkMaxConfig turningSparkMaxConfig", "SparkMaxConfig turningMotorConfig"),
        (r"m_drivingPIDController.setFeedbackDevice\(m_drivingEncoder\);", "drivingMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);"),
        (r"m_turningPIDController.setFeedbackDevice\(m_turningAbsoluteEncoder\);", "turningMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);"),
        (r"drivingEncoderConfig._encoder_", "drivingSparkMaxConfig.encoder"),
        (r"turningRelativeEncoderConfig._encoder_", "turningMotorConfig.encoder"),
        (r"turningAbsoluteEncoderConfig._encoder_", "turningMotorConfig.absoluteEncoder"),
        (r"m_turningAbsoluteEncoder.setInverted", "turningMotorConfig.inverted"),
        (r"m_turningPIDController.closedLoop", "turningMotorConfig.closedLoop"),
        (r"turningSparkMaxConfig", "turningMotorConfig"),
        (r"drivingSparkMaxConfig", "drivingMotorConfig"),
        (r"m_drivingSparkMax.setIdleMode", "drivingMotorConfig.idleMode"),
        (r"m_turningSparkMax.setIdleMode", "turningMotorConfig.idleMode"),
    ]

    replacements_map["y2024/Crescendo/src/main/java/com/gos/crescendo2024/subsystems/ArmPivotSubsystem.java"] = [
        ("//SparkMaxConfig pivotMotorConfig = new SparkMaxConfig\(\);", "SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();"),
        ("//SparkMaxConfig followMotorConfig = new SparkMaxConfig\(\);", "SparkMaxConfig followMotorConfig = new SparkMaxConfig();"),
        (r"pivotMotorEncoderConfig._encoder_", "pivotMotorConfig.encoder"),
        (r"pivotAbsEncoderConfig._encoder_", "pivotMotorConfig.absoluteEncoder"),
        (r"m_pivot.absoluteEncoder", "pivotMotorConfig.absoluteEncoder"),
        (r"m_sparkPidController.closedLoop", "pivotMotorConfig.closedLoop"),
        (r"m_sparkPidController.setFeedbackDevice\(m_pivotAbsEncoder\);", "pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);"),
        (r"m_sparkPidController.setFeedbackDevice\(m_pivotMotorEncoder\);", "pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);"),
    ]

    replacements_map["y2024/Crescendo/src/main/java/com/gos/crescendo2024/subsystems/HangerSubsystem.java"] = [
        ("//SparkMaxConfig leftHangerMotorConfig = new SparkMaxConfig\(\);", "SparkMaxConfig leftHangerMotorConfig = new SparkMaxConfig();"),
        ("//SparkMaxConfig rightHangerMotorConfig = new SparkMaxConfig\(\);", "SparkMaxConfig rightHangerMotorConfig = new SparkMaxConfig();"),
    ]

    replacements_map["y2024/Crescendo/src/main/java/com/gos/crescendo2024/subsystems/ShooterSubsystem.java"] = [
        (r"m_pidController.setFeedbackDevice\(m_shooterEncoder\);", "shooterMotorLeaderConfig.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);"),
    ]

    replacements_map["old_robots/y2022/RapidReact/src/main/java/com/gos/rapidreact/subsystems/CollectorSubsystem.java"] = [
        ("pivotNeoEncoderLeftConfig._encoder_", "pivotLeftConfig.encoder"),
        ("pivotNeoEncoderRightConfig._encoder_", "pivotRightConfig.encoder"),
        ("pivotExternalEncoderLeftConfig._encoder_", "pivotLeftConfig.alternateEncoder"),
        ("pivotExternalEncoderRightConfig._encoder_", "pivotRightConfig.alternateEncoder"),
    ]

    replacements_map["old_robots/y2020/2020InfiniteRecharge/src/main/java/com/gos/infinite_recharge/subsystems/Chassis.java"] = [
        ("leftEncoderConfig._encoder_", "masterLeftConfig.encoder"),
        ("rightEncoderConfig._encoder_", "masterRightConfig.encoder"),
        ("m_leftPidController.setOutputRange", "masterLeftConfig.closedLoop.outputRange"),
        ("m_rightPidController.setOutputRange", "masterRightConfig.closedLoop.outputRange"),
        ("m_leftPidController.setOutputRange", "masterLeftConfig.closedLoop.outputRange"),
        ("m_leftPidController.setSmartMotionAllowedClosedLoopError", "masterLeftConfig.closedLoop.smartMotion.allowedClosedLoopError"),
        ("m_rightPidController.setSmartMotionAllowedClosedLoopError", "masterRightConfig.closedLoop.smartMotion.allowedClosedLoopError"),
        ("m_leftPidController.setSmartMotionMinOutputVelocity", "masterLeftConfig.closedLoop.smartMotion.minOutputVelocity"),
        ("m_rightPidController.setSmartMotionMinOutputVelocity", "masterRightConfig.closedLoop.smartMotion.minOutputVelocity"),
    ]

    replacements_map["old_robots/y2020/2020InfiniteRecharge/src/main/java/com/gos/infinite_recharge/subsystems/Shooter.java"] = [
        ("m_pidController.setP", "masterConfig.closedLoop.p"),
        ("m_pidController.setD", "masterConfig.closedLoop.d"),
        ("m_pidController.setFF", "masterConfig.closedLoop.velocityFF"),
        (r"m_encoder.setInverted\(true\);", "masterConfig.encoder.inverted(true);"),
    ]

    replacements_map["old_robots/y2020/2020InfiniteRecharge/src/main/java/com/gos/infinite_recharge/subsystems/ShooterConveyor.java"] = [
        ("m_pidController.setP", "masterConfig.closedLoop.p"),
    ]

    replacements_map["old_robots/y2022/RapidReact/src/main/java/com/gos/rapidreact/subsystems/HangerSubsystem.java"] = [
        ("leftEncoderConfig._encoder_", "leftHangerConfig.encoder"),
        ("rightEncoderConfig._encoder_", "rightHangerConfig.encoder"),
    ]

    replacements_map["old_robots/y2023/ChargedUp/src/main/java/com/gos/chargedup/subsystems/TankDriveChassisSubsystem.java"] = [
        ("leftEncoderConfig._encoder_", "leaderLeftConfig.encoder"),
        ("rightEncoderConfig._encoder_", "leaderRightConfig.encoder"),
    ]

    replacements_map["old_robots/y2023/ChargedUp/src/main/java/com/gos/chargedup/subsystems/ArmPivotSubsystem.java"] = [
        ("absoluteEncoderConfig._encoder_", "pivotMotorConfig.absoluteEncoder"),
        ("pivotMotorEncoderConfig._encoder_", "pivotMotorConfig.encoder"),
        ("m_sparkPidController.closedLoop", "pivotMotorConfig.closedLoop"),
        (r"m_sparkPidController.setFeedbackDevice\(m_absoluteEncoder\);", "pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);"),
        (r"m_sparkPidController.setFeedbackDevice\(m_pivotMotorEncoder\);", "pivotMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);"),
        ("m_absoluteEncoder.setInverted", "pivotMotorConfig.absoluteEncoder.inverted"),
        ("m_absoluteEncoder.setZeroOffset", "pivotMotorConfig.absoluteEncoder.zeroOffset"),
    ]

    replacements_map["old_robots/y2022/MEPI/src/main/java/com/scra/mepi/rapid_react/subsystems/DrivetrainSubsystem.java"] = [
        ("leftEncoderConfig._encoder_", "leftLeaderConfig.encoder"),
        ("rightEncoderConfig._encoder_", "rightLeaderConfig.encoder"),
    ]

    replacements_map["old_robots/y2022/RapidReact/src/main/java/com/gos/rapidreact/subsystems/ChassisSubsystem.java"] = [
        ("leftEncoderConfig._encoder_", "leaderLeftConfig.encoder"),
        ("rightEncoderConfig._encoder_", "leaderRightConfig.encoder"),
    ]

    for filename in replacements_map:
        replacements = replacements_map[filename]
        regex_replace_file(filename, replacements)


def run_all_replacements(auto_commit=True):
    run_standard_replacement(auto_commit=auto_commit)
    run_our_additional_replacements(auto_commit=auto_commit)
    run_rev_replacements(auto_commit=auto_commit)
    snowflake_rev_replacements(auto_commit=auto_commit)


if __name__ == "__main__":
    #  py -m libraries.scripts.updater.replace_old_names
    run_all_replacements(auto_commit=False)
