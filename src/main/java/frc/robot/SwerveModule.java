/* Big thanks to Team 364 for the base code. */

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.ModuleStateOptimizer;
import frc.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public final int moduleNumber;
    private final Rotation2d angleOffset;
    private Rotation2d lastAngle = Rotation2d.fromDegrees(0);

    private final CANSparkMax angleMotor;
    private final CANSparkMax driveMotor;
    private final CANcoder absoluteEncoder;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder angleEncoder;

    private final SparkPIDController drivePIDController;
    private final SparkPIDController anglePIDController;

    private final SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(
                    Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    /* Sim Caches (basically im lazy and don't want to use the rev physics sim) */
    private double simSpeedCache;
    private Rotation2d simAngleCache = Rotation2d.fromDegrees(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Absolute Encoder */
        absoluteEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePIDController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive motor */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePIDController = driveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState =
                ModuleStateOptimizer.optimize(
                        desiredState,
                        getState().angle); // Custom optimize command, since default WPILib optimize
        // assumes continuous controller which CTRE is not

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        simSpeedCache = desiredState.speedMetersPerSecond;
        simAngleCache = desiredState.angle;
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            // mDriveMotor_ctre.set(ControlMode.PercentOutput, percentOutput);
            driveMotor.set(percentOutput);
        } else {
            driveMotor
                    .getPIDController()
                    .setReference(
                            desiredState.speedMetersPerSecond,
                            ControlType.kVelocity,
                            0,
                            feedforward.calculate(desiredState.speedMetersPerSecond),
                            SparkPIDController.ArbFFUnits.kVoltage);
            // mDriveMotor.set(ControlMode.Velocity, desiredState.speedMetersPerSecond,
            // DemandType.ArbitraryFeedForward,
            // feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01))
                        ? lastAngle
                        : desiredState.angle; // Prevent rotating module if speed is less than 1%.
        // Prevents Jittering.
        // mAngleMotor_ctre.set(ControlMode.Position,
        // Conversions.degreesToFalcon(angle.getDegrees(),
        // Constants.DriveSubsystem.angleGearRatio));
        angleMotor.getPIDController().setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        if (Robot.isReal()) return Rotation2d.fromDegrees(angleEncoder.getPosition());
        return simAngleCache; // If sim.
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition().getValue());
    }

    private void resetToAbsolute() {
        angleEncoder.setPosition(getAbsoluteAngle().getDegrees() - angleOffset.getDegrees());
    }

    private void configAngleEncoder() {
//        absoluteEncoder.configFactoryDefault();
//        absoluteEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor() {
        angleMotor.restoreFactoryDefaults();
        angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_SMART_CURRENT_LIMIT);
        angleMotor.setSecondaryCurrentLimit(Constants.Swerve.ANGLE_SECONDARY_CURRENT_LIMIT);
        angleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
        angleMotor.setIdleMode(Constants.Swerve.ANGLE_NEUTRAL_MODE);

        angleEncoder.setPositionConversionFactor(
                (1
                                / Constants.Swerve.chosenModule
                                        .angleGearRatio) // We do 1 over the gear ratio because 1
                        // rotation of the motor is < 1 rotation of
                        // the module
                        * 360); // 1/360 rotations is 1 degree, 1 rotation is 360 degrees.
        resetToAbsolute();

        anglePIDController.setP(Constants.Swerve.ANGLE_KP);
        anglePIDController.setI(Constants.Swerve.ANGLE_KI);
        anglePIDController.setD(Constants.Swerve.ANGLE_KD);
        anglePIDController.setFF(Constants.Swerve.ANGLE_KF);

        // TODO: Adjust this latter after we know the pid loop is not crazy
        angleMotor.getPIDController().setOutputRange(-.25, .25);
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_SMART_CURRENT_LIMIT);
        driveMotor.setSecondaryCurrentLimit(Constants.Swerve.DRIVE_SECONDARY_CURRENT_LIMIT);
        driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
        driveMotor.setIdleMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);
        driveMotor.setOpenLoopRampRate(Constants.Swerve.OPEN_LOOP_RAMP);
        driveMotor.setClosedLoopRampRate(Constants.Swerve.CLOSED_LOOP_RAMP);

        driveEncoder.setVelocityConversionFactor(
                1 / Constants.Swerve.DRIVE_GEAR_RATIO // 1/gear ratio because the wheel spins slower than
                        // the motor.
                        * Constants.Swerve.WHEEL_CIRCUMFERENCE // Multiply by the circumference to get meters
                        // per minute
                        / 60); // Divide by 60 to get meters per second.
        driveEncoder.setPosition(0);

        drivePIDController.setP(Constants.Swerve.DRIVE_KP);
        drivePIDController.setI(Constants.Swerve.DRIVE_KI);
        drivePIDController.setD(Constants.Swerve.DRIVE_KD);
        drivePIDController.setFF(
                Constants.Swerve
                        .DRIVE_KF); // Not actually used because we specify our feedforward when we
        // set our speed.

        // TODO: Remove after we know the pid loop isn't wild
        drivePIDController.setOutputRange(-.5, .5);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Robot.isReal() ? driveEncoder.getVelocity() : simSpeedCache, getAngle());
    }
}
