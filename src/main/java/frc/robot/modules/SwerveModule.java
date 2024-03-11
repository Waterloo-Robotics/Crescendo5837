package frc.robot.modules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    /* Motors */
    public CANSparkMax drive_spark;
    public CANSparkMax steer_spark;

    /* Encoders */
    public CANcoder steer_cancoder;
    public RelativeEncoder drive_encoder;

    /* PID Controllers */
    public PIDController drive_controller;
    public PIDController angle_controller;

    private Rotation2d last_angle;

    public SwerveModuleState last_state;

    public SwerveModule(int steer_id, int drive_id, int angle_id) {
        drive_spark = new CANSparkMax(drive_id, MotorType.kBrushless);

        drive_encoder = drive_spark.getEncoder();

        double kWheelDiameter = Units.inchesToMeters(4.0);
        double kWheelCircumference = kWheelDiameter * Math.PI;
        double kDriveRatio = 6.12 / 1.0;
        double kDistancePerMotorRotation = kWheelCircumference / kDriveRatio;

        drive_encoder.setVelocityConversionFactor(kDistancePerMotorRotation);
        drive_encoder.setPositionConversionFactor(kDistancePerMotorRotation);

        steer_spark = new CANSparkMax(steer_id, MotorType.kBrushless);
        steer_spark.setInverted(true);
        steer_cancoder = new CANcoder(angle_id);

        drive_controller = new PIDController(0.02, 0, 0);

        angle_controller = new PIDController(0.004, 0, 0);
        angle_controller.setTolerance(1);
        angle_controller.enableContinuousInput(-180, 180);

        last_angle = Rotation2d.fromDegrees(get_raw_angle());
    }

    public void set_module_state(SwerveModuleState state) {
        /*
         * Optimize the state - this handles reversing direction and minimizes the
         * change in heading
         */
        // state = SwerveModuleState.optimize(state, last_angle);

        
        // double drive_output = MathUtil.clamp(drive_controller.calculate(drive_encoder.getVelocity(), state.speedMetersPerSecond), -0.05, 0.05);
        double drive_output = MathUtil.clamp(state.speedMetersPerSecond, -0.10, 0.10);
        double steer_output = MathUtil.clamp(angle_controller.calculate(get_raw_angle(), state.angle.getDegrees()), -0.5, 0.5);

        /* Set the new powers to the SPARK Max controllers */
        drive_spark.set(drive_output);
        steer_spark.set(steer_output);

        /* Update last angle for use next time */
        last_angle = state.angle;
        last_state = state;

    }

    public double get_raw_angle() {
        return steer_cancoder.getAbsolutePosition().getValue() * 360 - 180;
    }

    public SwerveModulePosition get_module_position() {
        return new SwerveModulePosition(drive_encoder.getPosition(), Rotation2d.fromDegrees(this.get_raw_angle()));
    }

}
