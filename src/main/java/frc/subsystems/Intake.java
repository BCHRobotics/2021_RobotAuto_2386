package frc.subsystems;

import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.Constants;

/**
 * Intake of the robot.
 * 
 * This is the mechanism on the robot that retrieves the balls from the ground
 * and puts them into the {@link Stager}. It has a arm that can go up and down,
 * this allows it to be compact and stored away until the match starts. It also
 * has a bar of mecanum wheels that guide the ball into the {@link Stager}.
 * 
 * @author Noah Tomkins
 * @author Luc Suzuki
 * @author Kyla Rowan
 */
public class Intake extends Subsystem {
    private static Intake instance;

    private RobotOutput robotOut;
    private SensorInput sensorIn;

    private double intakeSpeed;
    private double armSpeed;

    /**
     * Get the instance of the {@link Intake}
     * @return instance
     */
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    /**
     * Create a Intake and init the robot IO
     */
    private Intake() {
        robotOut = RobotOutput.getInstance();
        sensorIn = SensorInput.getInstance();
        this.firstCycle();
    }

    /**
     * Set the intake speed
     * @param output speed of the intake (>= -1 and <= 1)
     */
    public void setIntakeSpeed(double speed) {
        if (speed < -1 || speed > 1) throw new IllegalArgumentException("Speed must be >= -1 and <= 1");

        this.intakeSpeed = speed;
    }

    /**
     * Set the speed of the arm
     * @param speed speed of the arm (>= -1 and <= 1)
     */
    public void setArmSpeed(double speed) {
        if (speed < -1 || speed > 1) throw new IllegalArgumentException("Speed must be >= -1 and <= 1");

        if ((sensorIn.getIntakeArmEncoder() <= 0 && speed < 0) ||
            (sensorIn.getIntakeArmEncoder() >= 19 && speed > 0)) {
            armSpeed = 0;
        } else {
            armSpeed = speed;
        }
    }

    /**
     * First cycle of the Intake
     */
    @Override
    public void firstCycle() {
        
    }

    /**
     * Calculate the actions the Intake will do during operation
     */
    @Override
    public void calculate() {

        robotOut.setIntakeArm(armSpeed);
        robotOut.setIntakeBar(intakeSpeed);

    }

    /**
     * Turn off all motors for the Intake
     */
    @Override
    public void disable() {
        robotOut.setIntakeArm(0);
        robotOut.setIntakeBar(0);
    }


}
