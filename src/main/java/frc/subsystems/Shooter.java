package frc.subsystems;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.imaging.Limelight;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.Constants;
import frc.util.PID;

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
    private Limelight limelight;

    private double turretSpeed;
    private double wheelSpeed;
    private double wheelRpm;

    private PID turretPID;

    private ShooterWheelState currentWheelState = ShooterWheelState.PERCENT_OUTPUT;
    private ShooterTurretState currentTurretState = ShooterTurretState.PERCENT_OUTPUT;

    public enum ShooterWheelState {
        PERCENT_OUTPUT, VELOCITY     
    }

    public enum ShooterTurretState {
        PERCENT_OUTPUT, VISION
    }

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
        limelight = Limelight.getInstance();
        this.firstCycle();
    }

    public ShooterWheelState getWheelState() {
        return currentWheelState;
    }

    public void setWheelState(ShooterWheelState state) {
        this.currentWheelState = state;
    }

    public ShooterTurretState getTurretState() {
        return currentTurretState;
    }

    public void setTurretState(ShooterTurretState state) {
        this.currentTurretState = state;
    }

    /**
     * Set the speed of the turret
     * @param speed speed the turret turns (>= -1 and <= 1)
     */
    public void setTurretSpeed(double speed) {
        if (speed < -1 || speed > 1) throw new IllegalArgumentException("Speed must be >= -1 and <= 1");

        double position = sensorIn.getShooterTurretEncoder();

        if (speed > 0 && position > 0) {
            turretSpeed = (-0.000123 * (position*position) + 1) * speed;
        } else if (speed < 0 && position < 0) {
            turretSpeed = (-0.000123 * (position*position) + 1) * speed;
        } else {
            turretSpeed = speed;
        }
    }

    public void setWheelRpm(double rpm) {
        this.wheelRpm = rpm;
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

        if (currentWheelState == ShooterWheelState.VELOCITY) {
            robotOut.setShooterWheelRPM(wheelRpm);
        } else {
            robotOut.setShooterWheel(wheelSpeed);
        }

        if (currentTurretState == ShooterTurretState.VISION) {
            limelight.setLedMode(3);
            if (limelight.getTargetExists()) {
                if (limelight.getTargetX() > 1.75 || limelight.getTargetX() < -1.75) {
                    robotOut.setShooterTurret(limelight.getTargetX() * 0.025);
                } else {
                    robotOut.setShooterTurret(0);
                }
            } else {
                robotOut.setShooterTurret(0);
            }
        } else {
            robotOut.setShooterTurret(turretSpeed);
            limelight.setLedMode(1);
        }
    }

    /**
     * Turn off all motors for the Shooter
     */
    @Override
    public void disable() {
        robotOut.setShooterTurret(0);
    }

}
