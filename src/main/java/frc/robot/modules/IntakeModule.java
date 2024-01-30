package frc.robot.modules;

import com.revrobotics.CANSparkMax;

public class IntakeModule {

    public enum ModuleStates {
        UNKNOWN,
        EMPTY_HOME,
        WAIT_FOR_NOTE,
        NOTE_FOUND,
        READY_FOR_TRANSFER;
    }

    public enum RequestStates {
        DEPLOY_INTAKE,
        CANCEL_INTAKE;
    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE;
    }

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final ModuleStates initialState = ModuleStates.EMPTY_HOME;

    /* Create sub-modules */
    private IntakePositionModule intakePosition;
    private IntakeRollersModule intakeRollers;

    /* Class Constructor */
    public IntakeModule(CANSparkMax intakeMotorController) {
        this.intakeRollers = new IntakeRollersModule(intakeMotorController);
    }

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
