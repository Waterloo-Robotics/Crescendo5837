package frc.robot.modules;

public class NoteTransferModule {

    public enum ModuleStates {
        EMPTY,
        LOADED,
        SHOOT;
    }

    public enum RequestStates {
        LOAD,
        SHOOT;
    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE;
    }

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final ModuleStates initalState = ModuleStates.EMPTY;

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
