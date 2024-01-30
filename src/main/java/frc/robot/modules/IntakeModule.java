package frc.robot.modules;

public class IntakeModule extends StatesModule{

    public enum ModuleStates {
        PICKUP,
        POSITION_NOTE,
        LOADED,
        TRANSFER,
        EMPTY;
    }

    public enum RequestStates {
        PICKUP,
        TRANSFER;
    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE;
    }

    public final ModuleStates initialState = ModuleStates.EMPTY;

}
