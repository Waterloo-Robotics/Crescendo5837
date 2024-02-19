package frc.robot.lib.auto.modes;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import frc.robot.lib.auto.events.Event;
import frc.robot.lib.auto.events.FollowTrajectory;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;

public class FollowTraj extends AutoMode {
    private ArrayList<Event> events;
    private int i;

    public FollowTraj(PathPlannerTrajectory trajectory, Swerve swerve) {
        events = new ArrayList<Event>();
        events.add(new FollowTrajectory(trajectory, swerve));

        i = 0;
    }

    public void initialize() {
        events.get(i).initialize();
    }

    public void run() {
        if(i < events.size()) {
            Event currentEvent = events.get(i);
            if(currentEvent.isFinished()) {
                currentEvent.finish();
                i++;
                if(i < events.size()) {
                    events.get(i).initialize();
                }
            } else {
                currentEvent.update();
            }
        }
    }

    public boolean isFinished() {
        if(i >= events.size()) {
            return true;
        }
        return false;
    }

    public void finish() {}
}