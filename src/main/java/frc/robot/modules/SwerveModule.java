package frc.robot.modules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class SwerveModule {
    /* Motors */
    CANSparkMax drive_motor;
    CANSparkMax steer_motor;

    /* Encoders */
    CANcoder angle_enc;
    RelativeEncoder drive_enc;

    /* PID Controllers */
    PIDController angle_controller;

    public SwerveModule(CANSparkMax drive, CANSparkMax steer, CANcoder angle_m )
    {
        drive_motor = drive;
        drive_enc = drive.getEncoder();
        steer_motor = steer;
        angle_enc = angle_m;

        angle_controller = new PIDController(0.006, 0, 0);
        angle_controller.enableContinuousInput(0, 180);
    } 

    public double get_raw_angle()
    {
        return angle_enc.getAbsolutePosition().getValue() * 360; 
    }

    
}
