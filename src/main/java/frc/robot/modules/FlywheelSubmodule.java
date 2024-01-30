package frc.robot.modules;

public class FlywheelSubmodule {

    public enum moduleStates {
        STOPPED,
        SPIN_UP,
        AT_SPEED,
        SPIN_DOWN;

    }

    public enum requestStates {
        STOP,
        SPIN_UP;
    }

    public enum requestStatusEnum {
        IN_PROGRESS,
        COMPLETE;
    }

    public moduleStates currenState;
    public requestStates requestState;
    public moduleStates lastState;
    public requestStatusEnum requestStatus;

    public final moduleStates initalState = moduleStates.STOPPED;

}
