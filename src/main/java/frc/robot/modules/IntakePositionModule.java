package frc.robot.modules;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;

public class IntakePositionModule {

    public enum ModuleStates {
        UNKNOWN,
        HOME,
        DEPLOYED,
        TRANSFERRING;
    }

    public enum RequestStates {
        HOME,
        DEPLOYED
    }

    public enum RequestStatusEnum {
        IN_PROGRESS,
        COMPLETE;
    }

    public ModuleStates currentState = ModuleStates.HOME;
    public RequestStates requestedState = RequestStates.HOME;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    PneumaticHub pneumaticHub;
    DoubleSolenoid rightCylinder;
    DoubleSolenoid leftCylinder;

    Timer transistionTimer;


    public IntakePositionModule(PneumaticHub pneumaticHub) {

        this.pneumaticHub = pneumaticHub;
        this.rightCylinder = pneumaticHub.makeDoubleSolenoid(0, 1);
        this.leftCylinder = pneumaticHub.makeDoubleSolenoid(2, 3);

        this.transistionTimer = new Timer();

        this.currentState = ModuleStates.HOME;
        this.requestedState = RequestStates.HOME;
    }

    public final ModuleStates initialState = ModuleStates.UNKNOWN;

    public void request_state(RequestStates state) {
        this.requestedState = state;

        switch (state) {

            case HOME:
                this.currentState = ModuleStates.TRANSFERRING;
                this.transistionTimer.reset();
                this.transistionTimer.start();

                this.rightCylinder.set(DoubleSolenoid.Value.kForward);
                this.leftCylinder.set(DoubleSolenoid.Value.kForward);
                break;

            case DEPLOYED:
                this.currentState = ModuleStates.DEPLOYED;
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

            case DEPLOYED:
                this.rightCylinder.set(DoubleSolenoid.Value.kReverse);
                this.leftCylinder.set(DoubleSolenoid.Value.kReverse);
                break;

            case HOME:
                
                break;
            
            case TRANSFERRING:
                if (this.requestedState == RequestStates.HOME && this.transistionTimer.hasElapsed(3)) {
                    /* Stop the timer and say we're home */
                    transistionTimer.stop();
                    this.currentState = ModuleStates.HOME;
                } else if (this.requestedState == RequestStates.DEPLOYED && this.transistionTimer.hasElapsed(3)) {
                    /* Stop the timer and say we're deployed */
                    transistionTimer.stop();
                    this.currentState = ModuleStates.DEPLOYED;
                }

        }

    }

}
