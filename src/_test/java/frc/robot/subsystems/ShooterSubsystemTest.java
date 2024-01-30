package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import frc.robot.Constants.ShooterConstants;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class ShooterSubsystemTest {
    private ShooterSubsystem shooter;

    @Before
    public void setup() {
        shooter = new ShooterSubsystem();
    }

    @After
    public void cleanup() {
        shooter.close();
        shooter = null;
    }

    @Test
    public void chechThatShooterStarts() {
        shooter.start();
        assertEquals(ShooterConstants.kSpeed, shooter.getShooter().get(), 0);
    }

    @Test
    public void checkThatShooterReverses() {
        shooter.reverse();
        assertEquals(ShooterConstants.kReverseSpeed, shooter.getShooter().get(), 0);
    }

    @Test
    public void checkThatShooterStops() {
        shooter.start();
        assertTrue(shooter.getShooter().get() != 0);
        shooter.stop();
        assertEquals(0, shooter.getShooter().get(), 0);
    }
}
