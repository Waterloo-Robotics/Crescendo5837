package frc.robot.subsystems;

import static org.junit.Assert.assertTrue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.*;

public class IntakeSubsystemTest {
    IntakeSubsystem intakeSubsystem;
    CANSparkMax intakeMotor;

    @Before
    public void setup() {
        intakeSubsystem = new IntakeSubsystem();
        intakeMotor = intakeSubsystem.getIntakeMoter();
        REVPhysicsSim.getInstance().addSparkMax(intakeMotor, DCMotor.getNeo550(1));
    }

    @After
    public void cleanup() {
        intakeSubsystem.close();
        intakeSubsystem = null;
        intakeMotor = null;
    }

    @Test
    public void checkThatIntakeStarts() {
        intakeSubsystem.start();
        assertTrue(intakeMotor.get() != 0.0);
    }

    public void checkThatIntakeStops() {
        intakeSubsystem.stop();
        assertTrue(intakeMotor.get() == 0.0);
    }
}
