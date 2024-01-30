package frc.robot.modules;

public class IntakeModule {

    public enum ModuleStates {
        PICKUP,
        POSITION_NOTE,
        LOADED,
        TRANSFER,
        EMPTY
    }

    public enum RequestStates {
        PICKUP,
        TRANSFER
    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE
    }

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final ModuleStates initialState = ModuleStates.EMPTY;

}
