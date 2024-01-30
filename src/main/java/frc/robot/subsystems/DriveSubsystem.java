/* Big thanks to Team 364 for the base swerve code. */

package frc.robot.subsystems;

import static frc.robot.Constants.Swerve.*;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.BlitzSubsystem;
import frc.robot.SwerveModule;

public class DriveSubsystem extends SubsystemBase implements BlitzSubsystem {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    private final ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("DriveSubsystem");

    private final Field2d field = new Field2d();

    public DriveSubsystem() {
        gyro = new AHRS();
        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(KINEMATICS, getYaw());

        mSwerveMods =
                new SwerveModule[] { // front left, front rigt, back left, back right.
                    new SwerveModule(FL, Mod0.CONSTANTS),
                    new SwerveModule(FR, Mod1.CONSTANTS),
                    new SwerveModule(BL, Mod2.CONSTANTS),
                    new SwerveModule(BR, Mod3.CONSTANTS)
                };
        initTelemetry();
    }

    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
                KINEMATICS.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(), translation.getY(), rotation, getYaw())
                                : new ChassisSpeeds(
                                        translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    // Use in above method?
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    //    public void park

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public Rotation2d getYaw() {
        return (invertGyro)
                ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModuleStates());
    }

    @Override
    public void simulationPeriodic() {
        drawRobotOnField(field);
    }

    public void initTelemetry() {
        for (SwerveModule mod : mSwerveMods) {
            shuffleboardTab.addNumber(
                    "Mod " + mod.moduleNumber + " CanCoder",
                    () -> mod.getAbsoluteAngle().getDegrees());
            shuffleboardTab.addNumber(
                    "Mod " + mod.moduleNumber + " Integrated",
                    () -> mod.getState().angle.getDegrees());
            shuffleboardTab.addNumber(
                    "Mod " + mod.moduleNumber + " Rotation",
                    () -> mod.getState().angle.getDegrees());
            shuffleboardTab.addNumber(
                    "Mod " + mod.moduleNumber + " Velocity",
                    () -> mod.getState().speedMetersPerSecond);
        }
        shuffleboardTab.add(field);
    }

    public void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
        // Draw a pose that is based on the robot pose, but shifted by the translation of the module
        // relative to robot center,
        // then rotated around its own center by the angle of the module.
        SwerveModuleState[] swerveModuleStates = getModuleStates();
        field.getObject("frontLeft")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(FL),
                                                swerveModuleStates[FL].angle)));
        field.getObject("frontRight")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(FR),
                                                swerveModuleStates[FR].angle)));
        field.getObject("backLeft")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(BL),
                                                swerveModuleStates[BL].angle)));
        field.getObject("backRight")
                .setPose(
                        getPose()
                                .transformBy(
                                        new Transform2d(
                                                CENTER_TO_MODULE.get(BR),
                                                swerveModuleStates[BR].angle)));
    }
}
