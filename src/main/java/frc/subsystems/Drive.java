package frc.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.Constants;
import frc.util.Lib;
import frc.util.PID;
import frc.util.PIDF;
import frc.util.Point;

public class Drive extends Subsystem {
    private static Drive instance;

    public enum DriveState {
        OUTPUT, 
        VELOCITY
    }

    private RobotOutput robotOutput;
    private SensorInput sensorInput;

    // states
    private DriveState currentState = DriveState.OUTPUT;
    private double leftOut;
    private double rightOut;

    // PID
    private PIDF leftVelPID;
    private PIDF rightVelPID;
    private PID straightPID;
    private PID turnPID;

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    private Drive() {
        this.robotOutput = RobotOutput.getInstance();
        this.sensorInput = SensorInput.getInstance();

        this.firstCycle();
    }

    private Point getRotatedError(double theta, double desiredX, double desiredY) {
        double currentX = this.sensorInput.getDriveXPos();
        double currentY = this.sensorInput.getDriveYPos();
        double rotation = 90 - theta;

        Point currentPosition = new Point(currentX, currentY);
        Point finalPosition = new Point(desiredX, desiredY);

        currentPosition.rotateByAngleDegrees(rotation);
        finalPosition.rotateByAngleDegrees(rotation);

        double xError = finalPosition.getX() - currentPosition.getX();
        double yError = finalPosition.getY() - currentPosition.getY();

        return new Point(xError, yError);
    }

    @Override
    public void firstCycle() {
        this.straightPID = new PID(Constants.getDriveStraightPID());
        this.straightPID.setMinDoneCycles(1);
        this.straightPID.setMinDoneCycles(10);
        this.straightPID.setIRange(1);

        this.leftVelPID = new PIDF(Constants.getDriveVelocityPID());
        this.leftVelPID.setMaxOutput(1);

        this.rightVelPID = new PIDF(Constants.getDriveVelocityPID());
        this.rightVelPID.setMaxOutput(1);

        this.turnPID = new PID(Constants.getDriveTurnPID());
        this.turnPID.setMinDoneCycles(10);
        this.turnPID.setMaxOutput(10);
        this.turnPID.setIRange(10);
    }

    /**
     * Sets output to drive
     * @param y percent output [-1 to 1] for forward movment
     * @param turn percent output [-1 to 1] for turn movment
     */
    public void setOutput(double y, double turn) {
        this.currentState = DriveState.OUTPUT;

        this.leftOut = (y + turn) * Constants.MAXOUTPUT;
        this.rightOut =  (y - turn) * Constants.MAXOUTPUT;
    }

    public void setTargetVelocity(double targetVel) {
        this.leftVelPID.setDesiredValue(targetVel);
        this.rightVelPID.setDesiredValue(targetVel);
    }

    public void driveAtVelocity(double velocity) {
        this.currentState = DriveState.VELOCITY;
        this.setTargetVelocity(velocity);
        double left = this.leftVelPID.calcPID(this.sensorInput.getDriveSpeedFPS());
        double right = this.rightVelPID.calcPID(this.sensorInput.getDriveSpeedFPS());
        this.robotOutput.setDriveLeft(left);
        this.robotOutput.setDriveRight(right);
    }

    public void setVelocityOutput(double leftOut, double rightOut) {
        this.currentState = DriveState.VELOCITY;

        this.leftVelPID.setDesiredValue(leftOut);
        this.rightVelPID.setDesiredValue(rightOut);

        rightOut = this.rightVelPID.calcPID(this.sensorInput.getDriveRightSpeedFPS());
        leftOut = this.leftVelPID.calcPID(this.sensorInput.getDriveLeftSpeedFPS());

        this.robotOutput.setDriveLeft(leftOut);
        this.robotOutput.setDriveRight(rightOut);
    }

    public void setRampRate(double rate) {
        this.robotOutput.setDriveRampRate(rate);
    }

    @Override
    public void calculate() {
        SmartDashboard.putString("DRIVE_STATE", this.currentState.toString());

        if (this.currentState == DriveState.OUTPUT) {
            this.robotOutput.setDriveLeft(this.leftOut);
            this.robotOutput.setDriveRight(this.rightOut);
        } else if (this.currentState == DriveState.VELOCITY) {
            this.robotOutput.setDriveLeft(this.leftOut);
            this.robotOutput.setDriveRight(this.rightOut);
        }
    }

	public boolean DriveToPoint(double x, double y, double theta, double minVelocity, double maxVelocity,
		double turnRate, double maxTurn, double eps) {
		this.straightPID.setMinMaxOutput(minVelocity, maxVelocity);
		Point error = getRotatedError(theta, x, y);
		double targetHeading;
		this.straightPID.setFinishedRange(eps);
		this.turnPID.setMaxOutput(10);

		if (error.getY() < 0) { // flip X if we are going backwards
			error.setX(-error.getX());
		}

		double turningOffset = (error.getX() * turnRate); // based on how far we are in x turn more
		
		if (turningOffset > maxTurn) { 
			turningOffset = maxTurn;
		} else if (turningOffset < -maxTurn) {
			turningOffset = -maxTurn;
		}
		
		targetHeading = theta - turningOffset;

		double angle = sensorInput.getGyroAngle();

		this.turnPID.setDesiredValue(targetHeading);

		

		double yError = error.getY();
		double yOutput;

		if (Math.abs(yError) > 3.0) {
			this.robotOutput.setDriveRampRate(0.20);
		} else {
			this.robotOutput.setDriveRampRate(0);
		}

		yOutput = this.straightPID.calcPIDError(yError);
		

		double distanceFromTargetHeading = Math.abs(this.turnPID.getDesiredVal() - angle);
		if (distanceFromTargetHeading > 90) { // prevents the y output from being reversed in the next calculation
			distanceFromTargetHeading = 90;
		}

		yOutput = yOutput * (((-1 * distanceFromTargetHeading) / 90.0) + 1);

		double xOutput = -this.turnPID.calcPID(angle);
	

		double leftOut = Lib.calcLeftTankDrive(xOutput, yOutput);
		double rightOut = Lib.calcRightTankDrive(xOutput, yOutput);

		this.setVelocityOutput(leftOut, rightOut);

		double dist = (yError);
		if (this.straightPID.isDone()) {
			System.out.println("I have reached the epsilon!");
		}

		boolean isDone = false;
		if (minVelocity <= 0.5) {
			if (this.straightPID.isDone()) {
				disable();
				isDone = true;
				this.robotOutput.setDriveRampRate(0);
			}
		} else if (Math.abs(dist) < eps) {
			isDone = true;
			this.robotOutput.setDriveRampRate(0);
		}

		return isDone;
	}

    @Override
    public void disable() {
        this.robotOutput.setDriveLeft(0.0);
        this.robotOutput.setDriveRight(0.0);
    }
    
}
