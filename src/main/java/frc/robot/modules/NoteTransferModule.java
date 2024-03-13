package frc.robot.modules;

import javax.swing.text.AbstractDocument.BranchElement;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class NoteTransferModule {

    public enum ModuleStates {
        STOPPED,
        SHOOT;
    }

    public enum RequestStates {
        STOP,
        SHOOT;
    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE;
    }

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final ModuleStates initalState = ModuleStates.STOPPED;

    public CANSparkMax transfer_spark;

    public NoteTransferModule(int transfer_can_id) {
        this.transfer_spark = new CANSparkMax(transfer_can_id, CANSparkLowLevel.MotorType.kBrushless);
        
        this.currentState = ModuleStates.STOPPED;
        this.requestedState = RequestStates.STOP;
    }

    public void request_state(RequestStates state) {
        this.requestedState = state;

        switch (state){
            case STOP:
                this.currentState = ModuleStates.STOPPED;
                break;
            case SHOOT:
                this.currentState = ModuleStates.SHOOT;
                break;
            default:
                /* Do nothing */
        }
    }

    public RequestStatusEnum get_request_status() {
        return this.requestStatus;
    }

    public ModuleStates get_state() {
        return this.currentState;
    }

    public void update() {

        switch(this.currentState) {
            case STOPPED:
                transfer_spark.set(0);
                break;
            case SHOOT:
                transfer_spark.set(1);
                break;
            default:
                /* Do nothing */
        }
    }

}
