package com.gos.scra.wcd.subsystems;


import com.gos.scra.wcd.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChassisSubsystem extends SubsystemBase {

    private final SparkMax m_leaderLeft;
    private final SparkMax m_followerLeft;

    private final SparkMax m_leaderRight;
    private final SparkMax m_followerRight;

    private final DifferentialDrive m_drive;

    public ChassisSubsystem() {
        m_leaderLeft = new SparkMax(Constants.DRIVE_LEFT_LEADER_SPARK, MotorType.kBrushless);
        m_followerLeft = new SparkMax(Constants.DRIVE_LEFT_FOLLOWER_SPARK, MotorType.kBrushless);
        m_leaderRight = new SparkMax(Constants.DRIVE_RIGHT_LEADER_SPARK, MotorType.kBrushless);
        m_followerRight = new SparkMax(Constants.DRIVE_RIGHT_FOLLOWER_SPARK, MotorType.kBrushless);

        SparkMaxConfig leaderLeftConfig = new SparkMaxConfig();
        SparkMaxConfig followerLeftConfig = new SparkMaxConfig();
        SparkMaxConfig leaderRightConfig = new SparkMaxConfig();
        SparkMaxConfig followerRightConfig = new SparkMaxConfig();

        leaderLeftConfig.idleMode(IdleMode.kCoast);
        followerLeftConfig.idleMode(IdleMode.kCoast);
        leaderRightConfig.idleMode(IdleMode.kCoast);
        followerRightConfig.idleMode(IdleMode.kCoast);

        m_leaderLeft.setInverted(true);
        m_leaderRight.setInverted(false);

        followerLeftConfig.follow(m_leaderLeft);
        followerRightConfig.follow(m_leaderRight);

        m_drive = new DifferentialDrive(m_leaderLeft, m_leaderRight);

        m_leaderLeft.configure(leaderLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_followerLeft.configure(followerLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_leaderRight.configure(leaderRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_followerRight.configure(followerRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setCurvatureDrive(double speed, double steer, boolean allowTurnInPlace) {
        m_drive.curvatureDrive(speed, steer, allowTurnInPlace);
    }

    public void setArcadeDrive(double speed, double steer) {
        m_drive.arcadeDrive(speed, steer);
    }
}
