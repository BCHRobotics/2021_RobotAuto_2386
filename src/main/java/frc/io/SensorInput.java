package frc.io;

import com.revrobotics.CANEncoder;

import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.util.Navx;

public class SensorInput {
    private static SensorInput instance;

    private RobotOutput robotOutput;

    private boolean firstCycle = true;	
    private double gyroAngle;
	private double lastGyroAngle;

    private Navx navx;
    private CANEncoder driveL1Encoder;
    private CANEncoder driveL2Encoder;
    private CANEncoder driveR1Encoder;
    private CANEncoder driveR2Encoder;

    private double xPosition = 0;
    private double yPosition = 0;

    private double lastTime = 0.0;
	private double deltaTime = 20.0;

    public static SensorInput getInstance() {
        if (instance == null) {
            instance = new SensorInput();
        }
        return instance;
    }

    private SensorInput() {
        this.robotOutput = RobotOutput.getInstance();

        this.navx = new Navx();

        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            this.driveL1Encoder = robotOutput.getDriveL1Encoder();
            this.driveL2Encoder = robotOutput.getDriveL2Encoder();
            this.driveR1Encoder = robotOutput.getDriveR1Encoder();
            this.driveR2Encoder = robotOutput.getDriveR2Encoder();
    
            this.driveL1Encoder.setPositionConversionFactor(1);
            this.driveL2Encoder.setPositionConversionFactor(1);
            this.driveR1Encoder.setPositionConversionFactor(1);
            this.driveR2Encoder.setPositionConversionFactor(1);
        } else if (Constants.CURRENT_ROBOT == RobotType.MINIBOT) {
            this.driveL1Encoder = robotOutput.getDriveL1Encoder();
            this.driveR1Encoder = robotOutput.getDriveR1Encoder();

            this.driveL1Encoder.setPositionConversionFactor(1);
            this.driveR1Encoder.setPositionConversionFactor(1);

            this.driveL1Encoder.setVelocityConversionFactor(0.34188);
        }

        this.reset();
    }

    public void reset() {
        this.firstCycle = true;
        this.navx.reset();

        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            this.driveL1Encoder.setPosition(0);
            this.driveL2Encoder.setPosition(0);
            this.driveR1Encoder.setPosition(0);
            this.driveR2Encoder.setPosition(0);
        } else if (Constants.CURRENT_ROBOT == RobotType.MINIBOT) {
            this.driveL1Encoder.setPosition(0);
            this.driveR1Encoder.setPosition(0);
        }
        
    }

    public void update() {

        if (this.lastTime == 0.0) {
            this.deltaTime = 20;
            this.lastTime = System.currentTimeMillis();
        } else {
            this.deltaTime = System.currentTimeMillis() - lastTime;
            this.lastTime = System.currentTimeMillis();
        }


        this.navx.update();

        double driveXSpeed = getDriveSpeedFPS() * Math.cos(Math.toRadians(getGyroAngle()));
        double driveYSpeed = getDriveSpeedFPS() * Math.sin(Math.toRadians(getGyroAngle()));
        xPosition += driveXSpeed * this.deltaTime / 1000.0;
        yPosition += driveYSpeed * this.deltaTime / 1000.0;
    }

    public double getDriveL1Encoder() {
        return this.driveL1Encoder.getPosition();
    }

    public double getDriveL2Encoder() {
        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            return this.driveL2Encoder.getPosition();
        }
        return 0;
    }

    public double getDriveR1Encoder() {
        return this.driveR1Encoder.getPosition();
    }

    public double getDriveR2Encoder() {
        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            return this.driveR2Encoder.getPosition();
        } 
        return 0;
    }

    public double getDriveL1SpeedFPS() {
        return (this.driveL1Encoder.getVelocity() * (Constants.getDriveWheelDiameter() * Math.PI) / 12) / 60;
    }

    public double getDriveL2SpeedFPS() {
        return (this.driveL2Encoder.getVelocity() * (Constants.getDriveWheelDiameter() * Math.PI) / 12) / 60;
    }

    public double getDriveR1SpeedFPS() {
        return (this.driveR1Encoder.getVelocity() * (Constants.getDriveWheelDiameter() * Math.PI) / 12) / 60;
    }

    public double getDriveR2SpeedFPS() {
        return (this.driveR2Encoder.getVelocity() * (Constants.getDriveWheelDiameter() * Math.PI) / 12) / 60;
    }

    public double getDriveLeftSpeedFPS() {
        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            return (getDriveL1SpeedFPS() + getDriveL2SpeedFPS()) / 2.0;
        } else if (Constants.CURRENT_ROBOT == RobotType.MINIBOT) {
            return getDriveL1SpeedFPS();
        }
        return 0;
    }

    public double getDriveRightSpeedFPS() {
        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            return (getDriveR1SpeedFPS() + getDriveR2SpeedFPS()) / 2.0;
        } else if (Constants.CURRENT_ROBOT == RobotType.MINIBOT) {
            return getDriveR1SpeedFPS();
        }
        return 0;
    }

    public double getDriveSpeedFPS() {
        return (getDriveLeftSpeedFPS() + getDriveRightSpeedFPS()) / 2.0;
    }

    public double getGyroAngle() {
        return this.navx.getAngle();
    }

    public void setDriveXPos(double x) {
        this.xPosition = x;
    }

    public void setDriveYPos(double y) {
        this.yPosition = y;
    }

    public double getDriveXPos() {
        return this.xPosition;
    }

    public double getDriveYPos() {
        return this.yPosition;
    }

}
