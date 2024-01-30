package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavigationSubsystem extends SubsystemBase {

    private final DriveSubsystem driveSubsystem;

    public NavigationSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }
}
