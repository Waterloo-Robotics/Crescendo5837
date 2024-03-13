package frc.robot.modules;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;

public class IntakeModule {

    public enum ModuleStates {
        UNKNOWN,
        EMPTY_HOME,
        INTAKING_NOTE,
        STORING_NOTE,
        READY_FOR_SHOT,
        SHOOTING,
        EMPTYING_INTAKE;
    }

    public enum RequestStates {
        DEPLOY_INTAKE,
        STORE_NOTE,
        EMPTY_INTAKE,
        SHOOT,
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
    public IntakePositionModule intakePosition;
    public IntakeRollersModule intakeRollers;

    DigitalInput noteDetectors = new DigitalInput(0);

    /* Class Constructor */
    public IntakeModule(int intakeMotorID, PneumaticHub pneumaticHub) {

        this.intakeRollers = new IntakeRollersModule(intakeMotorID);
        this.intakePosition = new IntakePositionModule(pneumaticHub);

    }

    public void request_state(RequestStates state) {
        this.requestedState = state;

        switch (state) {

            case DEPLOY_INTAKE:
                /* Tell the rollers to start intaking and put the intake down into deployed state */
                intakeRollers.request_state(IntakeRollersModule.RequestStates.INTAKE_NOTE);
                intakePosition.request_state(IntakePositionModule.RequestStates.DEPLOYED);
                this.currentState = ModuleStates.INTAKING_NOTE;
                break;
            case STORE_NOTE:
            /* We've just picked up a note */
                intakeRollers.request_state(IntakeRollersModule.RequestStates.STOP);
                intakePosition.request_state(IntakePositionModule.RequestStates.HOME);

                this.currentState = ModuleStates.STORING_NOTE;
                break;
            case EMPTY_INTAKE:
                intakeRollers.request_state(IntakeRollersModule.RequestStates.EMPTY_INTAKE);
                intakePosition.request_state(IntakePositionModule.RequestStates.HOME);
                this.currentState = ModuleStates.EMPTYING_INTAKE;
                break;

            case SHOOT:
                intakeRollers.request_state(IntakeRollersModule.RequestStates.TRANSFER_NOTE);
                intakePosition.request_state(IntakePositionModule.RequestStates.HOME);
                this.currentState = ModuleStates.SHOOTING;
                break;

            default:
            case CANCEL_INTAKE:
                this.currentState = ModuleStates.EMPTY_HOME;
                intakeRollers.request_state(IntakeRollersModule.RequestStates.STOP);
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

        switch (currentState) {

            case EMPTY_HOME:
                /* Do nothing */
                break;

            case INTAKING_NOTE:
            /* Intake is down and running */

                /* If we've picked up a note */
                if (this.noteDetectors.get()) {                  
                    /* Tell the intake module to store the note */
                    request_state(RequestStates.STORE_NOTE);

                }
                
                break;
            
            case STORING_NOTE:
            /* A note has just been picked up, we're stopped the intake and request it go to the 
             * home position. Wait until the intake position says its in the home position then 
             * start the note positioning process
             */
                /* Once the intake position module tells us that its home */
                if (intakePosition.get_state() == IntakePositionModule.ModuleStates.HOME)
                {
                    /* Store the note */
                    intakeRollers.request_state(IntakeRollersModule.RequestStates.POSITION_NOTE);
                    this.currentState = ModuleStates.READY_FOR_SHOT;
                }
                break;
            
            case READY_FOR_SHOT:
                /* If we're ready for a shot we don't need to do anything, just wait until we're requested to shoot */
                break;

            case SHOOTING:
                /* Do nothing  */
                break;

            case EMPTYING_INTAKE:
                /* The intake rollers will be in the emptying intake state until its empty then it will be in 
                 * the stopped state
                 */
                if (intakeRollers.get_state() == IntakeRollersModule.ModuleStates.STOPPED)
                {
                    this.currentState = ModuleStates.EMPTY_HOME;
                }
                break;

        }

        intakeRollers.update();
        intakePosition.update();

        lastState = currentState;

    }

}
