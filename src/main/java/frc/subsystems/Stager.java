package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.RobotOutput;
import frc.io.SensorInput;

/**
 * Stager of the robot.
 * 
 * This is the mechanism on the robot that holds and stores balls to shoot.
 * The stager allows for you to input balls from the {@link Intake} 
 * and output the balls to the {@link Shooter}
 * 
 * @author Noah Tomkins
 * @author Luc Suzuki
 * @author Kyla Rowan
 */
public class Stager extends Subsystem {
    private static Stager instance;

    private RobotOutput robotOut;
    private SensorInput sensorIn;

    /**
     * State of the stager, changing the state will change the operation the stager will do.
     * @see Stager
     */
    public enum StagerState {
        LOAD, UNLOAD, REVERSE_UNLOAD, LOAD_FULL_SPEED, UNLOAD_FULL_SPEED, DELAY_UNLOAD, DELAY_UNLOAD_AUTO, STOP
    }

    private StagerState currentState = StagerState.STOP;
    private double speed;

    private long delayStart;
    
    private double[] stagerOutput = new double[3];

    /**
     * Get the instance of the {@link Stager}
     * @return instance
     */
    public static Stager getInstance() {
        if (instance == null) {
            instance = new Stager();
        }
        return instance;
    }

    /**
     * Create a Stager and init the robot IO
     */
    private Stager() {
        robotOut = RobotOutput.getInstance();
        sensorIn = SensorInput.getInstance();
        this.firstCycle();
    }

    /**
     * Set the speed to input and output the balls
     * @param speed speed to input and output (>= 0 and <= 1)
     */
    public void setSpeed(double speed) {
        if (speed < 0 || speed > 1) throw new IllegalArgumentException("speed must be >= 0 and <= 1");

        this.speed = speed;
    }

    /**
     * Set the state of the stager
     * @param state state to set
     * @see StagerState
     */
    public void setState(StagerState state) {
        this.currentState = state;
    }

    /**
     * Get the current state of the stager
     * @return state of stager
     * @see StagerState
     */
    public StagerState getState() {
        return this.currentState;
    }

    /**
     * Load the balls into the stager at a specific speed
     * @param speed speed to load the balls (>= 0 and <= 1)
     */
    private void load(double speed) {
        if (speed < 0 || speed > 1) throw new IllegalArgumentException("speed must be >= 0 and <= 1");

        boolean[] balls = {
            sensorIn.getStagerSensor0(),
            sensorIn.getStagerSensor1(),
            sensorIn.getStagerSensor2()
        };

        stagerOutput[2] = (balls[2]) ? 0 : speed;
        stagerOutput[1] = (balls[1] && balls[2]) ? 0 : speed * 0.6;
        stagerOutput[0] = (balls[0] && balls[1] && balls[2]) ? 0 : speed * 0.6;
    }

    /**
     * Unload the balls into the shooter at speed
     * @param speed speed to unload balls (>= 0 and <= 1)
     */
    private void unload(double speed) {
        if (speed < 0 || speed > 1) throw new IllegalArgumentException("speed must be >= 0 and <= 1");

        for (int i = 0; i < stagerOutput.length; i++) {
            stagerOutput[i] = speed;
        }
    }

    /**
     * Unload the balls at a specific delay
     * @param delay ms to wait for each ball (>= 0)
     */
    private void delayedUnload(long delay) {
        if (delay < 0) throw new IllegalArgumentException("delay must be >= 0");

        if (System.currentTimeMillis() < delayStart + (delay*1)) {
            stagerOutput[0] = 0;
            stagerOutput[1] = 0;
            stagerOutput[2] = 1;
        } else if (System.currentTimeMillis() < delayStart + (delay*2)) {
            stagerOutput[0] = 0;
            stagerOutput[1] = 1 * 0.4;
            stagerOutput[2] = 1;
        } else if (System.currentTimeMillis() < delayStart + (delay*3)) {
            stagerOutput[0] = 1 * 0.4;
            stagerOutput[1] = 1 * 0.4;
            stagerOutput[2] = 1;
        }
    }

    /**
     * Unload the stager out the back not out the shooter.
     * This can be used when the balls are stuck in the stager
     * and need to be quickly expelled from the machine.
     * @param speed speed that the balls (>= 0 and <= 1)
     */
    private void reverseUnload(double speed) {
        if (speed < 0 || speed > 1) throw new IllegalArgumentException("Speed must be >= 0 and <= 1");

        for (int i = 0; i < stagerOutput.length; i++) {
            stagerOutput[i] = -speed;
        }
    }

    /**
     * First cycle of the stager
     */
    @Override
    public void firstCycle() {

    }

    /**
     * Calculate the actions the stager will do during operation
     */
    @Override
    public void calculate() {

        switch (currentState) {
            case LOAD:
                load(speed);
                break;
            case UNLOAD:
                unload(speed);
                break;
            case REVERSE_UNLOAD:
                reverseUnload(speed);
                break;
            case LOAD_FULL_SPEED:
                load(1);
                break;
            case UNLOAD_FULL_SPEED:
                unload(1);
                break;
            case DELAY_UNLOAD:
                delayedUnload(500);
                break;
            case DELAY_UNLOAD_AUTO:
                delayedUnload(2000);
                break;
            case STOP:
            default:
                stagerOutput[0] = 0;
                stagerOutput[1] = 0;
                stagerOutput[2] = 0;
                break;
        }

        if (currentState != StagerState.DELAY_UNLOAD && currentState != StagerState.DELAY_UNLOAD_AUTO) {
            delayStart = System.currentTimeMillis();
        }

        robotOut.setStager0(stagerOutput[0]);
        robotOut.setStager1(stagerOutput[1]);
        robotOut.setStager2(stagerOutput[2]);
        SmartDashboard.putNumber("stager0", stagerOutput[0]);
        SmartDashboard.putNumber("stager0", stagerOutput[1]);
        SmartDashboard.putNumber("stager0", stagerOutput[2]);
    }

    /**
     * Turn off all motors for the stager
     */
    @Override
    public void disable() {
        robotOut.setStager0(0);
        robotOut.setStager1(0);
        robotOut.setStager2(0);
    }
}
