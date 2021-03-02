package frc.io;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class RobotOutput {
    private static RobotOutput instance;

    // Drive motors
    private CANSparkMax driveL1;
    private CANSparkMax driveL2;
    private CANSparkMax driveR1;
    private CANSparkMax driveR2;

    /**
     * Get the instance of RobotOutput
     * @return instance
     */
    public static RobotOutput getInstance() {
        if (instance == null) {
            instance = new RobotOutput();
        }
        return instance;
    }

    /**
     * Creates a new RobotOutput and sets up the motors
     */
    private RobotOutput() {

        // init drive motors
        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            this.driveL1 = new CANSparkMax(11, MotorType.kBrushless);   
            this.driveL2 = new CANSparkMax(12, MotorType.kBrushless);   
            this.driveR1 = new CANSparkMax(15, MotorType.kBrushless);
            this.driveR2 = new CANSparkMax(16, MotorType.kBrushless);   
        } else if (Constants.CURRENT_ROBOT == RobotType.MINIBOT) {
            this.driveL1 = new CANSparkMax(11, MotorType.kBrushless);  
            this.driveR1 = new CANSparkMax(15, MotorType.kBrushless);
        }


        this.configureSpeedControllers();
    }

    /**
     * Configures the speed controllers for the robot,
     * this includes if the motors need to be inverted 
     * or current limited
     */
    public void configureSpeedControllers() {

        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
           

            this.driveL2.follow(this.driveL1, false);
            this.driveR2.follow(this.driveR1, false);

            this.driveL1.setSmartCurrentLimit(60, 10); 
            this.driveL2.setSmartCurrentLimit(60, 10);
            this.driveR1.setSmartCurrentLimit(60, 10);  
            this.driveR2.setSmartCurrentLimit(60, 10);
        } else if (Constants.CURRENT_ROBOT == RobotType.MINIBOT) {
            this.driveL1.setInverted(true);
            this.driveR1.setInverted(false);

            this.driveL1.setSmartCurrentLimit(60, 10); 
            this.driveR1.setSmartCurrentLimit(60, 10); 
        }   

    }


    /* Drive Motors */

    public void setDriveLeft(double output) {
        this.driveL1.set(output);
    }

    public void setDriveRight(double output) {
        this.driveR1.set(output);
    }

    public void setDriveRampRate(double rampRateSecondsToFull) {

        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            this.driveL1.setOpenLoopRampRate(rampRateSecondsToFull);
            this.driveL2.setOpenLoopRampRate(rampRateSecondsToFull);
            this.driveR1.setOpenLoopRampRate(rampRateSecondsToFull);
            this.driveR2.setOpenLoopRampRate(rampRateSecondsToFull);
        } else if (Constants.CURRENT_ROBOT == RobotType.MINIBOT) {
            this.driveL1.setOpenLoopRampRate(rampRateSecondsToFull);
            this.driveR1.setOpenLoopRampRate(rampRateSecondsToFull);
        }
        
    }

    public CANEncoder getDriveL1Encoder() {
        return this.driveL1.getEncoder();
    }

    public CANEncoder getDriveL2Encoder() {
        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            return this.driveL2.getEncoder();
        }
        return null;
    }

    public CANEncoder getDriveR1Encoder() {
        return this.driveR1.getEncoder();
    }

    public CANEncoder getDriveR2Encoder() {
        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            return this.driveR2.getEncoder();
        }
        return null;
    }

    /**
     * Stops all motors
     */
    public void stopAll() {
        setDriveLeft(0);
        setDriveRight(0);
    }

}
