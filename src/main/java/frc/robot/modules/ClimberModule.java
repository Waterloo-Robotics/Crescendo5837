package frc.robot.modules;

public class ClimberModule {

    public enum ModuleStates {
        NOT_HOMED,
        LEARN_IN_PROGRESS,
        LEARN_COMPLETE,
        LEARN_FAILED,
        HOME,
        DEPLOYED,
        ENGAGED,
        CLIMB;
    }

    public enum RequestStates {
        LEARN,
        HOME,
        DEPLOY,
        ENGAGE,
        CLIMB;
    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE,
        FAILED;
    }

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final ModuleStates initalState = ModuleStates.NOT_HOMED;

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
