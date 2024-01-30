package frc.robot.modules;

public class ClimberModule {

    public enum moduleStates {
        NOT_HOMED,
        LEARN_IN_PROGRESS,
        LEARN_COMPLETE,
        LEARN_FAILED,
        HOME,
        DEPLOYED,
        ENGAGED,
        CLIMB;
    }

    public enum requestStates {
        LEARN,
        HOME,
        DEPLOY,
        ENGAGE,
        CLIMB;
    }

    public enum requestStatusEnum {
        IN_PROGRESS,
        COMPLETE;
    }

    public moduleStates currenState;
    public requestStates requestState;
    public moduleStates lastState;
    public requestStatusEnum requestStatus;

    public final moduleStates initalState = moduleStates.NOT_HOMED;
    
}
