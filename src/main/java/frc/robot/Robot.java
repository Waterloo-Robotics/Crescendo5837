// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.constant.DirectMethodHandleDesc;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.modules.FlywheelSubmodule;
import frc.robot.modules.IntakeModule;
import frc.robot.modules.NoteTransferModule;
import frc.robot.modules.ShooterAngleModule;
import frc.robot.modules.SwerveBaseModule;
import frc.robot.modules.ShooterAngleModule.RequestStates;
import frc.robot.modules.ShooterModule;
import frc.robot.modules.SwerveBaseModule.DriveBaseStates;;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    // Drive Controllers
    // XboxController driver_controller = new XboxController(1);
    XboxController driver_controller = new XboxController(2);
    Joystick farmSim1 = new Joystick(0);
    // Joystick farmSim2 = new Joystick(5);


    // PDH
    PowerDistribution pdh = new PowerDistribution();

    SwerveBaseModule drivebase = new SwerveBaseModule(driver_controller);

    PneumaticHub pneumaticHub = new PneumaticHub(40);

    // CANdle led1 = new CANdle(45);
    // CANdle led2 = new CANdle(46);

    /* Create intake module */
    IntakeModule intake = new IntakeModule(24, pneumaticHub);

    /* Create note transfer module */
    NoteTransferModule note_transfer = new NoteTransferModule(19);

    /* Create flywheel module */
    FlywheelSubmodule flywheels = new FlywheelSubmodule(20, 21);

    /* Pivot module */
    ShooterAngleModule shooter_angle = new ShooterAngleModule(25);

    // 2 limelights
    // double tx = LimelightHelpers.getTX("");
    // double ty = LimelightHelpers.getTY("");
    // double area = LimelightHelpers.getTA("");
    // double tagID = LimelightHelpers.getFiducialID("");
    // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    // NetworkTableEntry tx = table.getEntry("tx");
    // NetworkTableEntry ty = table.getEntry("ty");
    // NetworkTableEntry ta = table.getEntry("ta");

    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    double last_shoot_trigger;

    double angle;

    Timer autoTimer = new Timer();

    private StructArrayPublisher<SwerveModuleState> publisher;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Do Nothing", kDefaultAuto);
        m_chooser.addOption("Do Stuff", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        drivebase.current_state = DriveBaseStates.XBOX;

        /* On robot init, dereference the shooter angle */
        shooter_angle.home_found = false;

        drivebase.gyro.reset();

        last_shoot_trigger = 0;

        publisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {

        // x = tx.getDouble(0.0);
        // y = ty.getDouble(0.0);
        // area = ta.getDouble(0.0);
        // tx = LimelightHelpers.getTX("");
        // ty = LimelightHelpers.getTY("");
        // area = LimelightHelpers.getTA("");
        // tagID = LimelightHelpers.getFiducialID("");

        // SmartDashboard.putNumber("LimelightTX", tx);
        // SmartDashboard.putNumber("LimelightTY", ty);
        // SmartDashboard.putNumber("LimelightArea", area);
        // SmartDashboard.putNumber("LimelightTagID", tagID);

        SmartDashboard.putString("Angle", shooter_angle.get_state().toString());
        SmartDashboard.putBoolean("Note Sensor", intake.noteDetectors.get());
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);

        /* If the shooter angle has not been homed, start the shooter angle in the homing sequence */
        if (!shooter_angle.home_found) {
            shooter_angle.request_state(RequestStates.FIND_HOME);
        }
        flywheels.request_state(FlywheelSubmodule.RequestStates.STOP);
        note_transfer.request_state(NoteTransferModule.RequestStates.STOP);

    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
                if (shooter_angle.get_state() == ShooterAngleModule.ModuleStates.HOMING){
                    
                } else if (shooter_angle.get_state() == ShooterAngleModule.ModuleStates.HOME) {
                    autoTimer.reset();
                    autoTimer.start();
                    shooter_angle.request_state(ShooterAngleModule.RequestStates.AMP_ANGLE);
                    flywheels.request_state(FlywheelSubmodule.RequestStates.SPIN_UP_SPEAKER);
                }

                if (autoTimer.hasElapsed(5)) {
                    note_transfer.request_state(NoteTransferModule.RequestStates.SHOOT);
                    intake.request_state(IntakeModule.RequestStates.SHOOT);
                }

                SmartDashboard.putNumber("Auto Time", autoTimer.get());

                shooter_angle.update();
                flywheels.update();
                note_transfer.update();
                intake.update();
                break;
            case kDefaultAuto:
            default:
                // Put default auto code here
                break;
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        pneumaticHub.enableCompressorAnalog(70, 110);

        /* Put the robot into driver mode */
        drivebase.current_state = DriveBaseStates.XBOX;

        /* If the shooter angle has not been homed, start the shooter angle in the homing sequence */
        if (!shooter_angle.home_found) {
            shooter_angle.request_state(RequestStates.FIND_HOME);
        }

        flywheels.request_state(FlywheelSubmodule.RequestStates.STOP);
        shooter_angle.request_state(ShooterAngleModule.RequestStates.HOME);
        note_transfer.request_state(NoteTransferModule.RequestStates.STOP);
        intake.request_state(IntakeModule.RequestStates.CANCEL_INTAKE);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        /* Gyro Rest */
        if (driver_controller.getStartButtonPressed()) {
            drivebase.gyro.reset();
        }

        /* Home All */
        /* 2 on Driver Controller or 21 on Farm sim 
         * 1 on Driver Controller released - ie no long pressing shoot
        */
        if (driver_controller.getYButton() || farmSim1.getRawButtonPressed(11) || 
            (driver_controller.getRightTriggerAxis() < 0.5 && last_shoot_trigger >= 0.5)) {
            /* Remove limit on drivebase speed */
            drivebase.set_max_drive_speed(1);
            intake.request_state(IntakeModule.RequestStates.CANCEL_INTAKE);
            note_transfer.request_state(NoteTransferModule.RequestStates.STOP);
            flywheels.request_state(FlywheelSubmodule.RequestStates.STOP);
            shooter_angle.request_state(ShooterAngleModule.RequestStates.HOME);
        }

        /* Intake mode */
        /* 7 on Driver Controller */
        if (driver_controller.getLeftTriggerAxis() > 0.5 || farmSim1.getRawButtonPressed(15)) {
            intake.request_state(IntakeModule.RequestStates.DEPLOY_INTAKE);
            note_transfer.request_state(NoteTransferModule.RequestStates.STOP);
            flywheels.request_state(FlywheelSubmodule.RequestStates.STOP);
            shooter_angle.request_state(ShooterAngleModule.RequestStates.HOME);
        }

        /* Eject */
        if (farmSim1.getRawButtonPressed(16)) {
            intake.request_state(IntakeModule.RequestStates.EMPTY_INTAKE);
        }

        /* Amp Prepare */
        /* 4 on Farm sim */
        if (farmSim1.getRawButtonPressed(4)) {
            /* Limit drivebase speed while flywheels are running */
            drivebase.set_max_drive_speed(0.5);
            note_transfer.request_state(NoteTransferModule.RequestStates.STOP);
            flywheels.request_state(FlywheelSubmodule.RequestStates.SPIN_UP_AMP);
            shooter_angle.request_state(ShooterAngleModule.RequestStates.AMP_ANGLE);
        }

        /* Speaker Prepare */
        /* 10 on Farm sim */
        if (farmSim1.getRawButtonPressed(10) || driver_controller.getLeftBumperPressed()) {
            /* Limit drivebase speed while flywheels are running */
            drivebase.set_max_drive_speed(0.3);
            note_transfer.request_state(NoteTransferModule.RequestStates.STOP);
            flywheels.request_state(FlywheelSubmodule.RequestStates.SPIN_UP_SPEAKER);
            /* Just using Amp angle for now since its likely going to be the same thing we if aren't using the camera */
            shooter_angle.request_state(ShooterAngleModule.RequestStates.AMP_ANGLE);
        }

        /* Speaker Far Prepare */
        /* 5 on Farm sim */
        if (farmSim1.getRawButtonPressed(5)) {
            /* Limit drivebase speed while flywheels are running */
            drivebase.set_max_drive_speed(0.3);
            note_transfer.request_state(NoteTransferModule.RequestStates.STOP);
            flywheels.request_state(FlywheelSubmodule.RequestStates.SPIN_UP_SPEAKER);
            /* Just using Amp angle for now since its likely going to be the same thing we if aren't using the camera */
            shooter_angle.request_state(ShooterAngleModule.RequestStates.SPEAKER_FAR);
        }

        /* Shoot */
        /* A on Driver Controller */
        if (driver_controller.getRightTriggerAxis() > 0.5 && last_shoot_trigger <= 0.5) {
            intake.request_state(IntakeModule.RequestStates.SHOOT);
            note_transfer.request_state(NoteTransferModule.RequestStates.SHOOT);
            /* Don't modify the flywheel state */
            /* Don't modify the shooter angle state */
        }

        last_shoot_trigger = driver_controller.getRightTriggerAxis();

        drivebase.update();
        intake.update();
        flywheels.update();
        note_transfer.update();
        shooter_angle.update();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // pneumaticHub.enableCompressorAnalog(70, 110);
        pneumaticHub.disableCompressor();
        shooter_angle.angle_encoder.setPosition(0);
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {

        // if (driver_controller.getAButton()) {
        //     SmartDashboard.putString("Button", "A");
        //     intake.request_state(IntakeModule.RequestStates.DEPLOY_INTAKE);
        // } else if (driver_controller.getBButton()) {
        //     SmartDashboard.putString("Button", "B");
        //     intake.request_state(IntakeModule.RequestStates.CANCEL_INTAKE);
        // } else if (driver_controller.getYButton()) {
        //     SmartDashboard.putString("Button", "Y");
        //     intake.request_state(IntakeModule.RequestStates.EMPTY_INTAKE);
        // }

        // /* Up on dpad */
        // if (driver_controller.getPOV() == 0) {
        //     flywheels.request_state(FlywheelSubmodule.RequestStates.SPIN_UP_AMP);
        // } 
        // /* Right on dpad */
        // else if (driver_controller.getPOV() == 90) {
        //     flywheels.request_state(FlywheelSubmodule.RequestStates.SPIN_UP_SPEAKER);
        // }
        // /* Down on dpad */
        // else if (driver_controller.getPOV() == 180) {
        //     flywheels.request_state(FlywheelSubmodule.RequestStates.STOP);
        // }

        // if (driver_controller.getStartButtonPressed()) {
        //     intake.request_state(IntakeModule.RequestStates.SHOOT);
        //     note_transfer.request_state(NoteTransferModule.RequestStates.SHOOT);
        // } else if (driver_controller.getStartButtonReleased()) {
        //     intake.request_state(IntakeModule.RequestStates.CANCEL_INTAKE);
        //     note_transfer.request_state(NoteTransferModule.RequestStates.STOP);
        // }

        // intake.update();
        // /* Run drivebase */
        // drivebase.update();
        // note_transfer.update();
        // flywheels.update();
        drivebase.update();
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
