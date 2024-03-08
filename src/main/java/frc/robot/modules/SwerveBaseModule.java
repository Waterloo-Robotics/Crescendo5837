package frc.robot.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveBaseModule {
    public SwerveModule[] modules;
    public SwerveModulePosition[] positions;

    public static final double kWheelOffset = Units.inchesToMeters(13.5);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d[] {
            new Translation2d(kWheelOffset, kWheelOffset),
            new Translation2d(-kWheelOffset, kWheelOffset),
            new Translation2d(kWheelOffset, -kWheelOffset),
            new Translation2d(-kWheelOffset, -kWheelOffset)
    });

    public SwerveDriveOdometry odometry;

    DriveBaseStates current_state;

    XboxController input_controller;

    public SwerveBaseModule(XboxController drive_controller) {
        /* Create the four swerve modules passing in each corner's CAN ID */
        this.modules = new SwerveModule[] {
                /* Front Left */
                new SwerveModule(2, 3, 4),
                /* Front Right */
                new SwerveModule(5, 6, 7),
                /* Rear Left */
                new SwerveModule(8, 9, 10),
                /* Rear Right */
                new SwerveModule(11, 12, 13)
        };

        this.positions = new SwerveModulePosition[4];
        positions[0] = modules[0].get_module_position();
        positions[1] = modules[1].get_module_position();
        positions[2] = modules[2].get_module_position();
        positions[3] = modules[3].get_module_position();

        this.odometry = new SwerveDriveOdometry(kinematics, null, positions);
    }

    private void drive_xbox() {
        /* Get the inputs from the controller */
        double x = input_controller.getLeftX();
        double y = input_controller.getLeftY();
        double rotation = input_controller.getRightX();

        /* Apply a deadband to prevent stick drift */
        x = MathUtil.applyDeadband(x, 0.1);
        y = MathUtil.applyDeadband(y, 0.1);
        rotation = MathUtil.applyDeadband(rotation, 0.1);

        /* Multiply each by max velocity to get desired velocity in each direction */
        double x_velocity_m_s = x * Units.feetToMeters(16);
        double y_velocity_m_s = y * Units.feetToMeters(16);
        double rotational_vel = rotation * 4;

        /*
         * Convert velocity in each axis to a general chassis velocity then use the
         * kinematics to convert the chassic velocity to individual swerve modules
         * "states" ie rotation and drive velocity.
         */
        ChassisSpeeds speeds = new ChassisSpeeds(x_velocity_m_s, y_velocity_m_s, rotational_vel);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        /* Send the new states to each swerve module */
        setModuleStates(states);
    }

    private void lock() {
        SwerveModuleState[] states = {
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };
        setModuleStates(states);
    }

    private void test_steer() {
        /* The goal of this function is to set every swerve module to the same angle */
        /* Get the inputs from the controller */
        double x = input_controller.getLeftX();
        double y = input_controller.getLeftY();

        /* Apply a deadband to prevent stick drift */
        x = MathUtil.applyDeadband(x, 0.1);
        y = MathUtil.applyDeadband(y, 0.1);

        double angle = Math.atan2(y, x);

        SwerveModuleState[] states = {
                new SwerveModuleState(0, Rotation2d.fromDegrees(angle)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(angle)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(angle)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(angle))
        };
        setModuleStates(states);

    }

    public void update() {
        switch (current_state) {
            case XBOX:
                drive_xbox();
                break;
            case LOCK:
                lock();
                break;
            case TEST_STEER:
                test_steer();
                break;
            default:
                lock();
        }

    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            modules[i].set_module_state(states[i]);
        }
    }

    public enum DriveBaseStates {
        XBOX,
        LOCK,
        TEST_STEER
    }
}
