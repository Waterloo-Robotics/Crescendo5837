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

public class SwerveModule {
    /* Motors */
    CANSparkMax drive_spark;
    CANSparkMax steer_spark;

    /* Encoders */
    CANcoder steer_cancoder;
    RelativeEncoder drive_encoder;

    /* PID Controllers */
    PIDController drive_controller;
    PIDController angle_controller;

    private Rotation2d last_angle;

    public SwerveModule(int steer_id, int drive_id, int angle_id) {
        drive_spark = new CANSparkMax(drive_id, MotorType.kBrushless);
        drive_encoder = drive_spark.getEncoder();
        steer_spark = new CANSparkMax(steer_id, MotorType.kBrushless);
        steer_cancoder = new CANcoder(angle_id);

        drive_controller = new PIDController(0, 0, 0);

        angle_controller = new PIDController(0.001, 0, 0);
        angle_controller.setTolerance(3);
        angle_controller.enableContinuousInput(-180, 180);

        last_angle = Rotation2d.fromDegrees(get_raw_angle());
    }

    public void set_module_state(SwerveModuleState state) {
        /*
         * Optimize the state - this handles reversing direction and minimizes the
         * change in heading
         */
        state = SwerveModuleState.optimize(state, last_angle);

        double drive_output = drive_controller.calculate(drive_encoder.getVelocity(), state.speedMetersPerSecond);
        double steer_output = MathUtil.clamp(angle_controller.calculate(get_raw_angle(), state.angle.getDegrees()), -0.05, 0.05);

        /* Set the new powers to the SPARK Max controllers */
        drive_spark.set(0);
        steer_spark.set(steer_output);

        /* Update last angle for use next time */
        last_angle = state.angle;

    }

    public double get_raw_angle() {
        return steer_cancoder.getAbsolutePosition().getValue() * 360 - 180;
    }

    public SwerveModulePosition get_module_position() {
        return new SwerveModulePosition(drive_encoder.getPosition(), Rotation2d.fromDegrees(this.get_raw_angle()));
    }

}
