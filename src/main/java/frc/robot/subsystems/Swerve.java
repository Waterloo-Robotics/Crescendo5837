package frc.robot.subsystems;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveModuleConstants.*;
import frc.robot.lib.swerve.SwerveGyro;
import frc.robot.lib.swerve.SwerveModule;

public class Swerve {
    private SwerveModule[] modules;
    private SwerveGyro gyro;
    private XboxController driveController;
    private DriveState wantedState, currentState;

    private SwerveModulePosition[] positions;
    private SwerveDriveOdometry odometry;

    private Timer trajTimer;
    private PathPlannerTrajectory trajectory;
    private PPHolonomicDriveController trajController;

    public Swerve(XboxController driveController) {
        this.modules = new SwerveModule[] {
            new SwerveModule(FL.kDriveID, FL.kDriveInverted, FL.kAngleID, FL.kAngleInverted, FL.kEncoderID, FL.kEncoderInverted, FL.kEncoderOffset),
            new SwerveModule(FR.kDriveID, FR.kDriveInverted, FR.kAngleID, FR.kAngleInverted, FR.kEncoderID, FR.kEncoderInverted, FR.kEncoderOffset),
            new SwerveModule(BL.kDriveID, BL.kDriveInverted, BL.kAngleID, BL.kAngleInverted, BL.kEncoderID, BL.kEncoderInverted, BL.kEncoderOffset),
            new SwerveModule(BR.kDriveID, BR.kDriveInverted, BR.kAngleID, BR.kAngleInverted, BR.kEncoderID, BR.kEncoderInverted, BR.kEncoderOffset)
        };
        this.gyro = new SwerveGyro(SwerveGyroConstants.kGyroOffset, SwerveGyroConstants.kGyroInverted);
        this.driveController = driveController;

        this.wantedState = DriveState.LOCK;
        this.currentState = DriveState.LOCK;
        
        this.positions = new SwerveModulePosition[4];
        positions[0] = modules[0].getModulePosition();
        positions[1] = modules[1].getModulePosition();
        positions[2] = modules[2].getModulePosition();
        positions[3] = modules[3].getModulePosition();

        this.odometry = new SwerveDriveOdometry(SwerveConstants.kinematics, gyro.getRotation2d(), positions);

        this.trajTimer = new Timer();
        this.trajController = new PPHolonomicDriveController(SwerveConstants.kTranslationConstants, SwerveConstants.kRotationConstants, SwerveConstants.kMaxVelocity, SwerveConstants.kModuleRadius);
    }

    public void driveXbox() {
        double x = driveController.getLeftX();
        x = MathUtil.applyDeadband(x, SwerveConstants.kDeadband);
        x *= SwerveConstants.kMaxVelocity;
        double y = driveController.getLeftY();
        y = MathUtil.applyDeadband(y, SwerveConstants.kDeadband);
        y *= SwerveConstants.kMaxVelocity;
        double rotation = driveController.getRightX();
        rotation = MathUtil.applyDeadband(rotation, SwerveConstants.kDeadband);
        rotation *= SwerveConstants.kMaxAngularVelocity;

        ChassisSpeeds speeds = new ChassisSpeeds(x, y, rotation);
        if(SwerveConstants.fieldCentric) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d());
        }

        SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public void stop() {
        ChassisSpeeds speeds = new ChassisSpeeds();
        SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    public void lock() {
        SwerveModuleState[] states = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        };
        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for(int i = 0; i < 4; i++) {
            modules[i].setModuleState(states[i]);
        }
    }

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        this.trajectory = trajectory;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public void setWantedState(DriveState wantedState) {
        if(this.wantedState != wantedState) {
            this.wantedState = wantedState;
            handleStateTransition();
        }
    }

    public void handleStateTransition() {
        switch(wantedState) {
            case TELEOP:
                break;
            case FOLLOW_TRAJ:
                trajTimer.reset();
                trajTimer.start();
                break;
            case LOCK:
                break;
        }
        currentState = wantedState;
    }

    public void update() {
        odometry.update(gyro.getRotation2d(), getModulePositions());
        switch(currentState) {
            case TELEOP:
                driveXbox();
                break;
            case FOLLOW_TRAJ:
                if(trajTimer.get() < trajectory.getTotalTimeSeconds()) {
                    PathPlannerTrajectory.State state = trajectory.sample(trajTimer.get());
                    ChassisSpeeds speeds = trajController.calculateRobotRelativeSpeeds(odometry.getPoseMeters(), state);
                    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
                    setModuleStates(states);
                } else {
                    setWantedState(DriveState.LOCK);
                }
                break;
            case LOCK:
                lock();
                break;
        }
    }

    public DriveState getCurrentState() {
        return currentState;
    }

    public enum DriveState {
        TELEOP,
        FOLLOW_TRAJ,
        LOCK
    }
}
