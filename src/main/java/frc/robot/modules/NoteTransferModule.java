package frc.robot.modules;

public class NoteTransferModule extends StatesModule {

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

    public final ModuleStates initalState = ModuleStates.EMPTY;

}
