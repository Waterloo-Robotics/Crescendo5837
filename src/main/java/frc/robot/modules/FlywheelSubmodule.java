package frc.robot.modules;

public class FlywheelSubmodule {

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

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final ModuleStates initalState = ModuleStates.STOPPED;

    public void request_state(RequestStates state) {
        this.requestedState = state;
    }

    public RequestStatusEnum get_request_status() {
        return this.requestStatus;
    }

    public ModuleStates get_state() {
        return this.currentState;
    }

}
