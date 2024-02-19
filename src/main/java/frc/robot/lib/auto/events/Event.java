package frc.robot.lib.auto.events;

public abstract class Event {
    public abstract void initialize();

    public abstract void update();

    public abstract boolean isFinished();

    public abstract void finish();
}