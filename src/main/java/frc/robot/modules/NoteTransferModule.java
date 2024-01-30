package frc.robot.modules;

public class NoteTransferModule {

    public enum moduleStates {
        EMPTY,
        LOADED,
        SHOOT;
    }

    public enum requestStates {
        LOAD,
        SHOOT;
    }

    public enum requestStatusEnum {
        IN_PROGRESS,
        COMPLETE;
    }

    public moduleStates currenState;
    public requestStates requestState;
    public moduleStates lastState;
    public requestStatusEnum requestStatus;

    public final moduleStates initalState = moduleStates.EMPTY;

}
