package frc.robot.lib.auto.events;

import edu.wpi.first.wpilibj.Timer;

public class WaitEvent extends Event {
    private Timer waitTimer;
    private double waitTime;

    public WaitEvent(double waitTime) {
        waitTimer = new Timer();
        this.waitTime = waitTime;
    }

    @Override
    public void initialize() {
        waitTimer.start();
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if(waitTimer.get() >= waitTime) {
            return true;
        }
        return false;
    }

    @Override
    public void finish() {
        waitTimer.stop();
    }
}