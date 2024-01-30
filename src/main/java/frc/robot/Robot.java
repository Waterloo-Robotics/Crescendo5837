/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs; // This needs to be fixed.
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        ctreConfigs = new CTREConfigs();
        robotContainer = new RobotContainer();

        System.out.println("Robot Start up at: " + Timer.getFPGATimestamp());
        StatusManager statusManager = StatusManager.getInstance();
        addPeriodic(statusManager, .2, .01);
    }

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /* ***** --- Autonomous --- ***** */

    // Called at the start of autonomous.
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule autonomous commands
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    // Called periodically during autonomous
    @Override
    public void autonomousPeriodic() {}

    // Called at the end of autonomous
    @Override
    public void autonomousExit() {
        // Cancel autonomous commands
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /* ***** --- Teleop --- ***** */

    // Called at the start of teleop
    @Override
    public void teleopInit() {
        System.out.println("TeleopInit");
    }

    // Called periodicly durring teleop
    @Override
    public void teleopPeriodic() {}

    // Called at the end of teleop.
    @Override
    public void teleopExit() {}

    /* ***** --- Test Mode --- ***** */

    // Called at the start of test mode
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    // Called periodicly durring test mode
    @Override
    public void testPeriodic() {}

    // Called at the end of test mode
    @Override
    public void testExit() {}

    /* ***** --- Disabled --- ***** */

    // Called when disabled
    @Override
    public void disabledInit() {}

    // Called periodicly when disabled
    @Override
    public void disabledPeriodic() {}

    // Called when the robot exits disabled mode
    @Override
    public void disabledExit() {}

    /* ***** --- Simulation --- ***** */

    // Called when the robot enters simulation
    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    // Called periodicly durring simulation
    @Override
    public void simulationPeriodic() {}
}
