package frc.robot.lib.auto.events;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.DriveState;

public class FollowTrajectory extends Event {
    private PathPlannerTrajectory trajectory;
    private Swerve swerve;

    public FollowTrajectory(PathPlannerTrajectory trajectory, Swerve swerve) {
        this.trajectory = trajectory;
        this.swerve = swerve;
    }

    public void initialize() {
        swerve.setTrajectory(trajectory);
        swerve.setWantedState(DriveState.FOLLOW_TRAJ);
    }

    public void update() {}
    
    public boolean isFinished() {
        if(swerve.getCurrentState() != DriveState.FOLLOW_TRAJ) {
            return true;
        }
        return false;
    }

    public void finish() {
        swerve.setWantedState(DriveState.LOCK);
    }
}