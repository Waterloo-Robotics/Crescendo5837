package frc.robot.modules;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;

public class IntakeModule {

    public enum ModuleStates {
        UNKNOWN,
        EMPTY_HOME,
        INTAKING_NOTE,
        NOTE_WAITING,
        ROLLER_RESETTING,
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

    public ModuleStates currentState = ModuleStates.EMPTY_HOME;
    public RequestStates requestedState = RequestStates.CANCEL_INTAKE;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public final ModuleStates initialState = ModuleStates.EMPTY_HOME;

    /* Create sub-modules */
    private IntakePositionModule intakePosition;
    public IntakeRollersModule intakeRollers;

    DigitalInput noteDetectors = new DigitalInput(0);

    Timer noteWaitTimer;

    /* Class Constructor */
    public IntakeModule(int intakeMotorID, PneumaticHub pneumaticHub) {

        this.intakeRollers = new IntakeRollersModule(intakeMotorID);
        this.intakePosition = new IntakePositionModule(pneumaticHub);
        this.noteWaitTimer = new Timer();

    }

    public void request_state(RequestStates state) {
        this.requestedState = state;

        switch (state) {

            case DEPLOY_INTAKE:
                this.currentState = ModuleStates.INTAKING_NOTE;
                break;

            default:
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

        switch (this.requestedState) {

            case DEPLOY_INTAKE:

                if (this.noteDetectors.get()) {

                    this.currentState = ModuleStates.NOTE_WAITING;
                    noteWaitTimer.reset();
                    noteWaitTimer.start();

                }
                intakePosition.request_state(IntakePositionModule.RequestStates.DEPLOYED);
                break;

            case CANCEL_INTAKE:
                break;

            default:
                break;

        }

        switch (currentState) {

            case EMPTY_HOME:
                intakeRollers.request_state(IntakeRollersModule.RequestStates.STOP);
                intakePosition.request_state(IntakePositionModule.RequestStates.HOME);
                break;

            case INTAKING_NOTE:
                intakeRollers.request_state(IntakeRollersModule.RequestStates.INTAKE_NOTE);
                intakePosition.request_state(IntakePositionModule.RequestStates.DEPLOYED);
                break;

            case NOTE_WAITING:
                intakeRollers.request_state(IntakeRollersModule.RequestStates.STOP);
                intakePosition.request_state(IntakePositionModule.RequestStates.HOME);
                if (noteWaitTimer.hasElapsed(5)) {

                    this.currentState = ModuleStates.ROLLER_RESETTING;

                }
                break;

            case ROLLER_RESETTING:
                intakeRollers.request_state(IntakeRollersModule.RequestStates.RESET);
                this.currentState = ModuleStates.NOTE_FOUND;
                break;

            case NOTE_FOUND:
                intakeRollers.request_state(IntakeRollersModule.RequestStates.POSITION_NOTE);
                intakePosition.request_state(IntakePositionModule.RequestStates.HOME);
                break;

        }

        intakeRollers.update();
        intakePosition.update();

        lastState = currentState;

    }

}
