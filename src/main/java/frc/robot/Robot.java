// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.constant.DirectMethodHandleDesc;

import com.ctre.phoenix.led.CANdle;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.modules.FlywheelSubmodule;
import frc.robot.modules.IntakeModule;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.modules.IntakeRollersModule;
import frc.robot.modules.NoteTransferModule;
import frc.robot.modules.SwerveBaseModule;
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
    XboxController driver_controller = new XboxController(1);
//    Joystick farmSim1 = new Joystick(4);
//    Joystick farmSim2 = new Joystick(5);


    // PDH
    PowerDistribution pdh = new PowerDistribution();

    XboxController xbox_controller = new XboxController(1);

    SwerveBaseModule drivebase = new SwerveBaseModule(xbox_controller);

    /* Shooter */
    static CANSparkMax shooterAngleNeo550 = new CANSparkMax(25, MotorType.kBrushless);

    PneumaticHub pneumaticHub = new PneumaticHub(40);

    // CANdle led1 = new CANdle(45);
    // CANdle led2 = new CANdle(46);

    /* Create intake module */
    IntakeModule intake = new IntakeModule(24, pneumaticHub);

    /* Create note transfer module */
    NoteTransferModule note_transfer = new NoteTransferModule(19);

    /* Create flywheel module */
    FlywheelSubmodule flywheels = new FlywheelSubmodule(20, 21);

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

    double angle;

    private StructArrayPublisher<SwerveModuleState> publisher;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        drivebase.current_state = DriveBaseStates.XBOX;

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

        SmartDashboard.putString("Intake State", intake.get_state().toString());
        SmartDashboard.putString("Intake Rollers State", intake.intakeRollers.get_state().toString());
        SmartDashboard.putString("Intake Position State", intake.intakePosition.get_state().toString());

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
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
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
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
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
        pneumaticHub.enableCompressorAnalog(70, 110);
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {

        SmartDashboard.putBoolean("A", driver_controller.getAButton());
        if (driver_controller.getAButton()) {
            SmartDashboard.putString("Button", "A");
            intake.request_state(IntakeModule.RequestStates.DEPLOY_INTAKE);
        } else if (driver_controller.getBButton()) {
            SmartDashboard.putString("Button", "B");
            intake.request_state(IntakeModule.RequestStates.CANCEL_INTAKE);
        } else if (driver_controller.getYButton()) {
            SmartDashboard.putString("Button", "Y");
            intake.request_state(IntakeModule.RequestStates.EMPTY_INTAKE);
        }

        /* Up on dpad */
        if (driver_controller.getPOV() == 0) {
            flywheels.request_state(FlywheelSubmodule.RequestStates.SPIN_UP_AMP);
        } 
        /* Right on dpad */
        else if (driver_controller.getPOV() == 90) {
            flywheels.request_state(FlywheelSubmodule.RequestStates.SPIN_UP_SPEAKER);
        }
        /* Down on dpad */
        else if (driver_controller.getPOV() == 180) {
            flywheels.request_state(FlywheelSubmodule.RequestStates.STOP);
        }

        if (driver_controller.getStartButtonPressed()) {
            intake.request_state(IntakeModule.RequestStates.SHOOT);
            note_transfer.request_state(NoteTransferModule.RequestStates.SHOOT);
        } else if (driver_controller.getStartButtonReleased()) {
            intake.request_state(IntakeModule.RequestStates.CANCEL_INTAKE);
            note_transfer.request_state(NoteTransferModule.RequestStates.STOP);
        }

        intake.update();
        /* Run drivebase */
        drivebase.update();
        note_transfer.update();
        flywheels.update();

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
