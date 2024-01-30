package frc.robot.modules;

public class DrivebaseModule {

    public enum ModuleStates {

    }

    public enum RequestStates {

    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE
    }

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final ModuleStates initialState = null;

}
