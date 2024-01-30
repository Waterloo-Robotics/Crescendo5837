package frc.robot.modules;

public class FlywheelSubmodule extends StatesModule {

    public enum ModuleStates {
        STOPPED,
        SPIN_UP,
        AT_SPEED,
        SPIN_DOWN,
        SATURATED_SPEED_NOT_MET;
    }

    public enum RequestStates {
        STOP,
        SPIN_UP;
    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE,
        INCOMPLETE;
    }

    public final ModuleStates initalState = ModuleStates.STOPPED;

}
