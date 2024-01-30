package frc.robot.modules;

public class ClimberModule extends StatesModule {

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

    public final ModuleStates initalState = ModuleStates.NOT_HOMED;
    
}
