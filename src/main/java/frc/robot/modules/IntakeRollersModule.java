package frc.robot.modules;

import com.revrobotics.CANSparkMax;

public class IntakeRollersModule {

    public enum ModuleStates {
        STOPPED,
        WAITING_FOR_NOTE,
        POSITIONING_NOTE,
        NOTE_IN_POSITION,
        TRANSFER_NOTE;
    }

    public enum RequestStates {
        STOP,
        NOTE_IN_POSITION,
        TRANSFER_NOTE;
    }

    public enum RequestStatusEnum {
        COMPLETE,
        IN_PROGRESS,
        FAILED;
    }

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final static ModuleStates initialState = ModuleStates.STOPPED;

    private CANSparkMax intakeRollerMotor;

    public IntakeRollersModule(CANSparkMax motorController) {
        this.intakeRollerMotor = motorController;
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
