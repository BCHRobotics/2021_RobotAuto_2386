package frc.subsystems;

import frc.io.RobotOutput;
import frc.io.SensorInput;

/**
 * Shooter of the robot.
 * 
 * This is the mechanism on the robot that shoots the balls into the target.
 * The Shooter takes the balls from {@link Stager} and uses a adjustable speed
 * fly wheel to change the path of the ball, it also has a turret which changes 
 * the direction the shooter is facing.
 * 
 * @author Noah Tomkins
 * @author Luc Suzuki
 * @author Kyla Rowan
 */
public class Shooter extends Subsystem {
    private static Shooter instance;

    private RobotOutput robotOut;
    private SensorInput sensorIn;

    private double turretSpeed;
    private double wheelSpeed;

    // when the turret gets past these ranges it will slow down
    private static final int minTurret = -75;
    private static final int minSafeTurret = -45;
    private static final int minVerySafeTurret = -15;
    private static final int maxTurret = 190;
    private static final int maxSafeTurret = 160;
    private static final int maxVerySafeTurret = 130;

    /**
     * Get the instance of the {@link Shooter}
     * @return instance
     */
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    /**
     * Create a Shooter and init the robot IO
     */
    private Shooter() {
        robotOut = RobotOutput.getInstance();
        sensorIn = SensorInput.getInstance();
        this.firstCycle();
    }

    /**
     * Set the speed of the turret
     * @param speed speed the turret turns (>= -1 and <= 1)
     */
    public void setTurretSpeed(double speed) {
        if (speed < -1 || speed > 1) throw new IllegalArgumentException("Speed must be >= -1 and <= 1");

        double position = sensorIn.getShooterTurretEncoder();

        if ((position <= minTurret && speed < 0) || (position >= maxTurret && speed > 0)) {
            turretSpeed = 0;
        } else if ((position <= minSafeTurret && speed < 0) || (position >= maxSafeTurret && speed > 0)) {
            turretSpeed = speed * 0.1;
        } else if ((position <= minVerySafeTurret && speed < 0) || (position >= maxVerySafeTurret && speed > 0)) {
            turretSpeed = speed * 0.2;
        } else {
            turretSpeed = speed;
        }
    }

    /**
     * Set the speed of the fly wheel
     * @param speed speed of the fly wheel (>= 0 and <= 1)
     */
    public void setWheelSpeed(double speed) {
        if (speed < 0 || speed > 1) throw new IllegalArgumentException("Speed must be >= 0 and <= 1");

        this.wheelSpeed = speed;
    }

    /**
     * First cycle of the shooter
     */
    @Override
    public void firstCycle() {

    }

    /**
     * Calculate the actions the Shooter will do during operation
     */
    @Override
    public void calculate() {
        robotOut.setShooterTurret(turretSpeed);
        robotOut.setShooterWheel(wheelSpeed);
    }

    /**
     * Turn off all motors for the Shooter
     */
    @Override
    public void disable() {
        robotOut.setShooterTurret(0);
    }
}
