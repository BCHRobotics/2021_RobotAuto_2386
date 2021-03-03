package frc.subsystems;

import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.Constants;

public class Intake extends Subsystem {
    private static Intake instance;

    private RobotOutput robotOut;
    private SensorInput sensorIn;

    private double intakeSpeed;
    private double armSpeed;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        robotOut = RobotOutput.getInstance();
        sensorIn = SensorInput.getInstance();
        this.firstCycle();
    }

    @Override
    public void firstCycle() {
        
    }

    public void setIntakeOutput(double output) {
        this.intakeSpeed = output;
    }

    public void setArmOutput(double output) {
        this.armSpeed = output;
    }


    @Override
    public void calculate() {

        if ((sensorIn.getIntakeArmEncoder() <= Constants.intakeArmMin && armSpeed < 0) ||
            (sensorIn.getIntakeArmEncoder() >= Constants.intakeArmMax && armSpeed > 0)) {
            robotOut.setIntakeArm(0);
        } else {
            robotOut.setIntakeArm(armSpeed);
        }

        robotOut.setIntakeBar(intakeSpeed);

    }

    @Override
    public void disable() {
        robotOut.setIntakeArm(0);
        robotOut.setIntakeBar(0);
    }


}
