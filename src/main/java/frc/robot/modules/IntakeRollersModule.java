package frc.robot.modules;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        INTAKE_NOTE,
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

    public CANSparkMax intakeRollerMotor;

    public IntakeRollersModule(int intakeMotorID) {
        this.intakeRollerMotor = new CANSparkMax(intakeMotorID, CANSparkLowLevel.MotorType.kBrushless);
    }

    public void request_state(RequestStates state) {
        this.requestedState = state;

        switch (state) {

            case INTAKE_NOTE:
                this.currentState = ModuleStates.WAITING_FOR_NOTE;
                break;

            case STOP:
                this.currentState = ModuleStates.STOPPED;
                break;

        }

    }

    public RequestStatusEnum get_request_status() {
        return this.requestStatus;
    }

    public ModuleStates get_state() {
        return this.currentState;
    }

    public void update() {

        SmartDashboard.putString("Intake Rollers Current State", String.valueOf(this.currentState));

        switch (this.currentState) {

            case WAITING_FOR_NOTE:
                intakeRollerMotor.set(0.5);
                break;

            case STOPPED:
                intakeRollerMotor.set(0);
                break;

        }

        lastState = currentState;

    }

}
