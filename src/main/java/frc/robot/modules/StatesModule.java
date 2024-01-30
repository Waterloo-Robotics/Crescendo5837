package frc.robot.modules;

public class StatesModule {

    public enum ModuleStates {

    }

    public enum RequestStates {

    }

    public enum RequestStatusEnum {

    }

    public ModuleStates currentState;
    public RequestStates requestedState;
    public ModuleStates lastState;
    public RequestStatusEnum requestStatus;

    public void request_state(RequestStates state) 
    {
        requestedState = state;
    }

    public RequestStatusEnum get_request_status()
    {
        return requestStatus;
    }
    
}
