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
            new Translation2d(kWheelOffset, -kWheelOffset),
            new Translation2d(-kWheelOffset, kWheelOffset),
            new Translation2d(-kWheelOffset, -kWheelOffset)
    });

    public SwerveDriveOdometry odometry;

    public DriveBaseStates current_state;

    private XboxController input_controller;
    private int lock_counter;
    private boolean lock;

    public SwerveBaseModule(XboxController drive_controller) {
        /* Create the four swerve modules passing in each corner's CAN ID */
        this.modules = new SwerveModule[] {
                /* Front Left */
                new SwerveModule(2, 3, 4, false),
                /* Front Right */
                new SwerveModule(5, 6, 7, false),
                /* Rear Left */
                new SwerveModule(8, 9, 10, false),
                /* Rear Right */
                new SwerveModule(11, 12, 13, false)
        };

        this.positions = new SwerveModulePosition[4];
        positions[0] = modules[0].get_module_position();
        positions[1] = modules[1].get_module_position();
        positions[2] = modules[2].get_module_position();
        positions[3] = modules[3].get_module_position();

        input_controller = drive_controller;
        lock_counter = 0;
        lock = false;

        // this.odometry = new SwerveDriveOdometry(kinematics, null, positions);
    }

    private void lock() {
        SwerveModuleState[] states = {
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };
        setModuleStates(states);
    }

    private void drive_xbox() {
        /* Get the inputs from the controller */
        double x = input_controller.getLeftY();
        double y = input_controller.getLeftX();
        double rotation = input_controller.getRightX();

        /* Apply a deadband to prevent stick drift */
        x = MathUtil.applyDeadband(x, 0.25);
        y = MathUtil.applyDeadband(y, 0.25);
        rotation = MathUtil.applyDeadband(rotation, 0.2);

        /* If no inputs are present, lock the drivebase */
        if (Math.abs(x) + Math.abs(y) + Math.abs(rotation) < 0.05) {
            if (lock) {
                lock();
            } else if (lock_counter > 50) {
                lock = true;
            } else {
                lock_counter++;
            }

        } else {
            lock_counter = 0;
            lock = false;
        }

        if (!lock) {
            /* Multiply each by max velocity to get desired velocity in each direction */
            double x_velocity_m_s = x * Units.feetToMeters(1);
            double y_velocity_m_s = y * Units.feetToMeters(1);
            double rotational_vel = rotation * 2;

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

    }

    private void straight() {
        SwerveModuleState[] states = {
                new SwerveModuleState(input_controller.getLeftY()*0.1, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(input_controller.getLeftY()*0.1, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(input_controller.getLeftY()*0.1, Rotation2d.fromDegrees(0)),
                new SwerveModuleState(input_controller.getLeftY()*0.1, Rotation2d.fromDegrees(0))
        };
        setModuleStates(states);
    }

    public void stop() {
        for (int i = 0; i < 4; i++) {
            modules[i].steer_spark.set(0);
            modules[i].drive_spark.set(0);
        }
    }

    private void test_steer() {
        /* The goal of this function is to set every swerve module to the same angle */
        /* Get the inputs from the controller */
        double x = input_controller.getLeftX();
        double y = input_controller.getLeftY();

        /* Apply a deadband to prevent stick drift */
        x = MathUtil.applyDeadband(x, 0.1);
        y = MathUtil.applyDeadband(y, 0.1);

        double angle = Math.atan2(y, x) * (180  / Math.PI);

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
            case STRAIGHT:
                straight();
                break;
            case STOP:
                stop();
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
        STRAIGHT,
        STOP,
        TEST_STEER
    }
}
