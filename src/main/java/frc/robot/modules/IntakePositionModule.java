package frc.robot.modules;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;

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

    public IntakePositionModule(PneumaticHub pneumaticHub) {

        this.pneumaticHub = pneumaticHub;
        this.rightCylinder = pneumaticHub.makeDoubleSolenoid(0, 1);
        this.leftCylinder = pneumaticHub.makeDoubleSolenoid(2, 3);

    }

    public final ModuleStates initialState = ModuleStates.UNKNOWN;

    public void request_state(RequestStates state) {
        this.requestedState = state;

        switch (state) {

            case HOME:
                this.currentState = ModuleStates.HOME;
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

        switch (requestedState) {

            case DEPLOYED:
                this.rightCylinder.set(DoubleSolenoid.Value.kReverse);
                this.leftCylinder.set(DoubleSolenoid.Value.kReverse);
                break;

            case HOME:
                this.rightCylinder.set(DoubleSolenoid.Value.kForward);
                this.leftCylinder.set(DoubleSolenoid.Value.kForward);

        }

    }

}
