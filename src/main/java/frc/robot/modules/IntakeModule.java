package frc.robot.modules;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.management.MBeanRegistration;

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
    public IntakeRollersModule intakeRollers;

    DigitalInput noteDetectors = new DigitalInput(0);

    XboxController input_device;

    /* Class Constructor */
    public IntakeModule(int intakeMotorID, XboxController input) {
        this.input_device = input;
        this.intakeRollers = new IntakeRollersModule(intakeMotorID);
    }

    public void request_state(RequestStates state) {
        this.requestedState = state;

        switch (state) {

            case DEPLOY_INTAKE:
                this.currentState = ModuleStates.WAIT_FOR_NOTE;
                break;

            case CANCEL_INTAKE:
                this.currentState = ModuleStates.EMPTY_HOME;
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

        SmartDashboard.putString("Intake Current State", String.valueOf(this.currentState));

        switch (this.currentState) {

            case WAIT_FOR_NOTE:
                intakeRollers.request_state(IntakeRollersModule.RequestStates.INTAKE_NOTE);
                break;

            case EMPTY_HOME:
                intakeRollers.request_state(IntakeRollersModule.RequestStates.STOP);
                break;

            default:
                SmartDashboard.putString("Yo", "Default");
                break;

        }

        intakeRollers.update();

        lastState = currentState;

    }

}
