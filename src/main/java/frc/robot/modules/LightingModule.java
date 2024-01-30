package frc.robot.modules;

public class LightingModule {

    public enum ModuleStates {
        IDLE,
        PICKUP,
        NOTE_FOUND,
        NOTE_SECURED,
        NOTE_TRANSFERRED,
        SHOOTER_READY,
        TARGET_FOUND,
        SHOOTER_IN_PROGRESS,
        SHOOTER_FAILED,
        TARGET_AMP,
        TARGET_SPEAKER,
        CLIMB_READY,
        CLIMB_ENGAGE,
        CLIMB_COMPLETE
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

    public final ModuleStates initialState = ModuleStates.IDLE;

}
