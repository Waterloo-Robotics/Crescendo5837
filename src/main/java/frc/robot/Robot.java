// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.InputConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.DriveState;

public class Robot extends TimedRobot {
  // private Swerve swerve;
  private XboxController driveController;

  // private CANSparkMax fl_drive = new CANSparkMax(3, MotorType.kBrushless);
  private CANSparkMax fl_steer = new CANSparkMax(2, MotorType.kBrushless);
  private CANcoder fl_angle = new CANcoder(4);

  private PIDController fl_controller = new PIDController(0.006, 0, 0);

  @Override
  public void robotInit() {
    driveController = new XboxController(InputConstants.kDriverControllerID);
    fl_controller.enableContinuousInput(0, 360);
    fl_controller.setTolerance(1);
    // swerve = new Swerve(driveController);
  }

  @Override
  public void robotPeriodic() {
    // SmartDashboard.putNumber("0", swerve.modules[0].angleController.getSetpoint());
    // SmartDashboard.putNumber("1", swerve.modules[1].angleController.getSetpoint());
    // SmartDashboard.putNumber("2", swerve.modules[2].angleController.getSetpoint());
    // SmartDashboard.putNumber("3", swerve.modules[3].angleController.getSetpoint());

    SmartDashboard.putNumber("Current Angle", fl_angle.getAbsolutePosition().getValue()*360);
    SmartDashboard.putNumber("Desired Angle", Math.atan2(driveController.getLeftY(), driveController.getLeftX()) * (180 / Math.PI)+180);
    SmartDashboard.putNumber("Error", fl_controller.getPositionError());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    // swerve.update();
  }

  @Override
  public void teleopInit() {
    // swerve.setWantedState(DriveState.TELEOP);
  }

  @Override
  public void teleopPeriodic() {
    // if (driveController.getAButton()) swerve.setWantedState(DriveState.LOCK);
    // if (driveController.getBButton()) swerve.setWantedState(DriveState.TELEOP);

    // swerve.update();
  }

  @Override
  public void disabledInit() {
    // swerve.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() 
  {
    double angle_deg = Math.atan2(driveController.getLeftY(), driveController.getLeftX()) * (180 / Math.PI) + 180;

    double power = fl_controller.calculate(fl_angle.getAbsolutePosition().getValue()*360, angle_deg);
    SmartDashboard.putNumber("power", power);

    if (driveController.getAButton())
    {
      fl_steer.set(power);
    }
    else
    {
      fl_steer.set(0);
    }

  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
