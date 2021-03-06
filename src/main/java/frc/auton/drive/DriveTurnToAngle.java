package frc.auton.drive;

import frc.auton.AutonCommand;
import frc.auton.RobotComponent;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.Constants;
import frc.subsystems.Drive;
import frc.util.Lib;
import frc.util.PID;

public class DriveTurnToAngle extends AutonCommand {

    private SensorInput sensorIn;
    private RobotOutput robotOut;
    private double targetAngle;
    private double eps;
    private PID turnPID;
    private double maxOutput;
    private Drive drive;

    public DriveTurnToAngle(double targetAngle, double eps, long timeoutLength) {
        this(targetAngle, 11, eps, timeoutLength);
    }

    public DriveTurnToAngle(double targetAngle, double maxOutput, double eps, long timeoutLength) {
        super(RobotComponent.DRIVE, timeoutLength);
        this.targetAngle = targetAngle;
        this.drive = Drive.getInstance();
        this.maxOutput = maxOutput;
        this.eps = eps;
        this.robotOut = RobotOutput.getInstance();
        this.sensorIn = SensorInput.getInstance();
    }

    @Override
    public void firstCycle() {
        this.turnPID = new PID(Constants.getDriveTurnPID());
        this.turnPID.setMaxOutput(this.maxOutput);  
        this.turnPID.setFinishedRange(this.eps);  
        this.turnPID.setDesiredValue(this.targetAngle);
        this.turnPID.setIRange(10);
    }

    @Override
    public boolean calculate() {
        double x = -this.turnPID.calcPID(this.sensorIn.getGyroAngle());
        if (x > this.maxOutput) {
            x = this.maxOutput;
        } else if (x < -this.maxOutput) {
            x = -this.maxOutput;
        }

        if (this.turnPID.isDone()) {
            this.robotOut.setDriveLeft(0);
            this.robotOut.setDriveRight(0);
            return true;
        } else {
            double leftOut = Lib.calcLeftTankDrive(x, 0);
            double rightOut = Lib.calcLeftTankDrive(x, 0);

            this.drive.setVelocityOutput(leftOut, -rightOut);
            return false;
        }
    }

    @Override
    public void override() {
        this.robotOut.setDriveLeft(0);
		this.robotOut.setDriveRight(0);
    }
    
}
