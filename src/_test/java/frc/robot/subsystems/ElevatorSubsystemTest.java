package frc.robot.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.spy;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.Utils;
import org.junit.*;

public class ElevatorSubsystemTest {
    private ElevatorSubsystem elevatorSubsystem;
    DigitalInput topLimit;
    DigitalInput bottomLimit;
    WPI_TalonFX master;
    WPI_TalonFX slave;

    @Before
    public void setup() {
        // I am not sure if the following line is nessesary
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        WPIUtilJNI.setMockTime(0); // Set mock time so we can simulate filters without waiting

        // This must be spy not mock. Mock makes void methods not do anything while this does not
        // disrupt their normal behavior
        master = spy(new WPI_TalonFX(ElevatorConstants.kMasterPort)); // Master
        slave = spy(new WPI_TalonFX(ElevatorConstants.kSlavePort)); // Slave

        topLimit = mock(DigitalInput.class);
        bottomLimit = mock(DigitalInput.class);

        when(topLimit.get()).thenReturn(false); // Disable limit switches
        when(bottomLimit.get()).thenReturn(false);

        elevatorSubsystem =
                new ElevatorSubsystem( // Call our other with these hardwere inputs
                        master, // Master spy
                        slave, // Slave spy
                        topLimit, // Mock top limit
                        bottomLimit // Mock bottom limit
                        );
    }

    @After
    public void cleanup() {
        elevatorSubsystem.close();
        elevatorSubsystem = null;
        master = null;
        slave = null;
        topLimit = null;
        bottomLimit = null;
    }

    @BeforeClass
    public static void startup() {
        WPIUtilJNI.enableMockTime();
    }

    @AfterClass
    public static void shutdown() {
        WPIUtilJNI.disableMockTime();
    }

    @Test
    public void checkThatSlaveFollowsMaster() {
        verify(slave).follow(master);
    }

    @Test
    public void checkThatOneMotorIsInverted() {
        // Assert that either the master or the slave is inverted, fail if both
        assertTrue(master.getInverted() ^ slave.getInverted());
    }

    @Test
    public void checkThatBothMotorsBreak() {
        verify(master).setNeutralMode(NeutralMode.Brake);
        verify(slave).setNeutralMode(NeutralMode.Brake);
    }

    @Test
    public void checkThatFactoryDefautsAreRestored() {
        verify(master).configFactoryDefault();
        verify(slave).configFactoryDefault();
    }

    @Test
    public void checkThatUpMakesElevatorGoUp() {
        elevatorSubsystem.upElevator();
        // We must simulate the passage of time.
        // this method does that for us and calls the command schedular at a regular rate
        Utils.mockRunScheduler(1); // We only need to do this once to test objects using slewrate

        // Check that the elevator is moving up
        assertTrue(
                master.get() >= Math.min(0, ElevatorConstants.kUpSpeed)
                        && master.get() <= Math.max(0, ElevatorConstants.kUpSpeed)
                        && master.get() != 0);
        // assertTrue(slave.get() >= Math.min(0, ElevatorConstants.kUpSpeed) && slave.get() <=
        // Math.max(0, ElevatorConstants.kUpSpeed) && slave.get() != 0);
    }

    @Test
    public void checkThatDownMakesElevatorGoDown() {
        elevatorSubsystem.downElevator();
        // We must simulate the passage of time.
        // this method does that for us and calls the command schedular at a regular rate
        Utils.mockRunScheduler(1); // We only need to do this once to test objects using slewrate

        // Check that the elevator is moving down
        assertTrue(
                "Checks Speed is within acceptable range",
                master.get() >= Math.min(0, ElevatorConstants.kDownSpeed)
                        && master.get() <= Math.max(0, ElevatorConstants.kDownSpeed)
                        && master.get() != 0);
        // assertTrue("Checks Speed is within acceptable range", slave.get() >= Math.min(0,
        // ElevatorConstants.kDownSpeed) && slave.get() <= Math.max(0, ElevatorConstants.kDownSpeed)
        // && slave.get() != 0);
    }

    @Test
    public void checkThatElevatorReachesTopUpSpeed() {
        elevatorSubsystem.upElevator();
        // We must simulate the passage of time.
        // this method does that for us and calls the command schedular at a regular rate
        Utils.mockRunScheduler(20); // 20 times to ensure slewrate allows us to get to max speed
        assertEquals(ElevatorConstants.kUpSpeed, master.get(), 0.01);
        // assertEquals(ElevatorConstants.kUpSpeed, slave.get(), 0.01);
    }

    @Test
    public void checkThatElevatorReachesTopDownSpeed() {
        elevatorSubsystem.downElevator();
        // We must simulate the passage of time.
        // this method does that for us and calls the command schedular at a regular rate
        Utils.mockRunScheduler(20); // 20 times to ensure slewrate allows us to get to max speed
        assertEquals(ElevatorConstants.kDownSpeed, master.get(), 0.01);
        // assertEquals(ElevatorConstants.kDownSpeed, slave.get(), 0.01);
    }

    @Test
    public void checkThatHaultInstantlyStopsElevator() {
        elevatorSubsystem.upElevator();
        Utils.mockRunScheduler(20);
        assertTrue(master.get() != 0); // Make sure we are moving
        elevatorSubsystem.haultElevator();
        // We must simulate the passage of time.
        // this method does that for us and calls the command schedular at a regular rate
        Utils.mockRunScheduler(
                1); // Command scheduler should only need to run once to stop the elevator
        assertEquals(0, master.get(), 0.0);
    }

    @Test
    public void checkThatStopEventualyStopsElevator() {
        elevatorSubsystem.upElevator();
        Utils.mockRunScheduler(20);
        assertTrue(master.get() != 0); // make sure we are moving
        elevatorSubsystem.stopElevator();
        // We must simulate the passage of time.
        // this method does that for us and calls the command schedular at a regular rate
        Utils.mockRunScheduler(20);
        assertEquals(0, master.get(), 0.0);
    }

    @Test
    public void checkThatTopLimitSwitchStopsElevatorGoingUp() {
        elevatorSubsystem.upElevator();
        Utils.mockRunScheduler(20);
        when(topLimit.get()).thenReturn(true); // Enable the limit switch
        Utils.mockRunScheduler(1);
        assertEquals(0, master.get(), 0.0);
    }

    @Test
    public void checkThatBottomLimitSwitchStopsElevatorGoingDown() {
        elevatorSubsystem.downElevator();
        Utils.mockRunScheduler(20);
        when(bottomLimit.get()).thenReturn(true); // Enable the limit switch
        Utils.mockRunScheduler(1);
        assertEquals(0, master.get(), 0.0);
    }

    @Test
    public void checkThatBottomLimitSwitchDoesNotStopElevatorFromGoingUp() {
        when(bottomLimit.get()).thenReturn(true);
        elevatorSubsystem.upElevator();
        Utils.mockRunScheduler(20);
        assertEquals(ElevatorConstants.kUpSpeed, master.get(), 0.0);
    }

    @Test
    public void checkThatTopLimitSwitchDoesNotStopElevatorFromGoingDown() {
        when(topLimit.get()).thenReturn(true);
        elevatorSubsystem.downElevator();
        Utils.mockRunScheduler(20);
        assertEquals(ElevatorConstants.kDownSpeed, master.get(), 0.0);
    }
}
