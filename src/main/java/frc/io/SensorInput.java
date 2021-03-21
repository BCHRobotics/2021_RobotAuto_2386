package frc.io;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.util.Navx;

public class SensorInput {
    private static SensorInput instance;

    private RobotOutput robotOut;

    private boolean firstCycle = true;	
    private double gyroAngle;
	private double lastGyroAngle;

    // Drive encoders
    private Navx navx;
    private CANEncoder driveL1Encoder;
    private CANEncoder driveL2Encoder;
    private CANEncoder driveR1Encoder;
    private CANEncoder driveR2Encoder;

    // Intake encoders
    private CANEncoder intakeArmEncoder;

    // Stager sensors
    private DigitalInput stagerSensor0;
    private DigitalInput stagerSensor1;
    private DigitalInput stagerSensor2;

    // Shooter encoders
    private CANEncoder shooterTurretEncoder;
    private CANEncoder shooterWheelEncoder;

    // Robot position
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
        this.robotOut = RobotOutput.getInstance();

        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            this.navx = new Navx(SerialPort.Port.kUSB);

            this.driveL1Encoder = robotOut.getDriveL1Encoder();
            this.driveL2Encoder = robotOut.getDriveL2Encoder();
            this.driveR1Encoder = robotOut.getDriveR1Encoder();
            this.driveR2Encoder = robotOut.getDriveR2Encoder();
    
            // when factor 1 travels: 46.4
            double driveFactor = 100 / 46.4;
            this.driveL1Encoder.setPositionConversionFactor(driveFactor);
            this.driveL2Encoder.setPositionConversionFactor(driveFactor);
            this.driveR1Encoder.setPositionConversionFactor(driveFactor);
            this.driveR2Encoder.setPositionConversionFactor(driveFactor);

            this.intakeArmEncoder = robotOut.getIntakeArmEncoder();
            this.intakeArmEncoder.setPositionConversionFactor(1);

            this.stagerSensor0 = new DigitalInput(4);
            this.stagerSensor1 = new DigitalInput(3);
            this.stagerSensor2 = new DigitalInput(0);

            this.shooterTurretEncoder = robotOut.getShooterTurretEncoder();
            this.shooterWheelEncoder = robotOut.getShooterWheelEncoder();
            this.shooterTurretEncoder.setPositionConversionFactor(1.162325);
            this.shooterWheelEncoder.setPositionConversionFactor(1);
        } else if (Constants.CURRENT_ROBOT == RobotType.MINIBOT) {
            this.navx = new Navx(SPI.Port.kMXP);

            this.driveL1Encoder = robotOut.getDriveL1Encoder();
            this.driveR1Encoder = robotOut.getDriveR1Encoder();

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

            this.intakeArmEncoder.setPosition(0);

            this.shooterTurretEncoder.setPosition(0);
            this.shooterWheelEncoder.setPosition(0);
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

    /* Drive */

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


    /* Intake */

    public double getIntakeArmEncoder() {
        return this.intakeArmEncoder.getPosition();
    }


    /* Stager */
    
    public boolean getStagerSensor0() {
        return !stagerSensor0.get();
    }

    public boolean getStagerSensor1() {
        return !stagerSensor1.get();
    }

    public boolean getStagerSensor2() {
        return !stagerSensor2.get();
    }

    /* Shooter */

    public double getShooterTurretEncoder() {
        return this.shooterTurretEncoder.getPosition();
    }

    public double getShooterWheelEncoder() {
        return this.shooterWheelEncoder.getPosition();
    }
}
