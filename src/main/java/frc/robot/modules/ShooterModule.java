package frc.robot.modules;

public class ShooterModule extends StatesModule {

    public enum ModuleStates {
        IDLE,
        AIMING,
        READY,
        SHOOT
    }

    public enum RequestStates {

    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE
    }

    public final ModuleStates initialState = ModuleStates.IDLE;

}
