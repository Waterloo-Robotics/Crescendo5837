package frc.robot;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    // public TalonFXConfiguration swerveAngleFXConfig;
    // public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        // swerveAngleFXConfig = new TalonFXConfiguration();
        // swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* DriveSubsystem Angle Motor Configurations */
        // SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.DriveSubsystem.angleEnableCurrentLimit,
        //     Constants.DriveSubsystem.angleContinuousCurrentLimit,
        //     Constants.DriveSubsystem.anglePeakCurrentLimit,
        //     Constants.DriveSubsystem.anglePeakCurrentDuration);

        // swerveAngleFXConfig.slot0.kP = Constants.DriveSubsystem.angleKP;
        // swerveAngleFXConfig.slot0.kI = Constants.DriveSubsystem.angleKI;
        // swerveAngleFXConfig.slot0.kD = Constants.DriveSubsystem.angleKD;
        // swerveAngleFXConfig.slot0.kF = Constants.DriveSubsystem.angleKF;
        // swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* DriveSubsystem Drive Motor Configuration */
        // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.DriveSubsystem.driveEnableCurrentLimit,
        //     Constants.DriveSubsystem.driveContinuousCurrentLimit,
        //     Constants.DriveSubsystem.drivePeakCurrentLimit,
        //     Constants.DriveSubsystem.drivePeakCurrentDuration);

        // swerveDriveFXConfig.slot0.kP = Constants.DriveSubsystem.driveKP;
        // swerveDriveFXConfig.slot0.kI = Constants.DriveSubsystem.driveKI;
        // swerveDriveFXConfig.slot0.kD = Constants.DriveSubsystem.driveKD;
        // swerveDriveFXConfig.slot0.kF = Constants.DriveSubsystem.driveKF;
        // swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        // swerveDriveFXConfig.openloopRamp = Constants.DriveSubsystem.openLoopRamp;
        // swerveDriveFXConfig.closedloopRamp = Constants.DriveSubsystem.closedLoopRamp;

        /* DriveSubsystem CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.CAN_CODER_INVERT;
        swerveCanCoderConfig.initializationStrategy =
                SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
