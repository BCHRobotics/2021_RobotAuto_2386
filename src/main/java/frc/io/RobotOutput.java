package frc.io;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
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

    // Intake motors
    private CANSparkMax intakeArm;
    private TalonSRX intakeBar;

    // Stager motors
    private TalonSRX stager0;
    private TalonSRX stager1;
    private TalonSRX stager2;

    // Shooter motors
    private CANSparkMax shooterTurret;
    private CANSparkMax shooterWheel;
    private CANPIDController shooterWheelPID;

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
            
            this.intakeArm = new CANSparkMax(25, MotorType.kBrushless);
            this.intakeBar = new TalonSRX(26);

            this.stager0 = new TalonSRX(30);
            this.stager1 = new TalonSRX(31);
            this.stager2 = new TalonSRX(32);

            this.shooterTurret = new CANSparkMax(21, MotorType.kBrushless);
            this.shooterWheel = new CANSparkMax(22, MotorType.kBrushless);
            this.shooterWheelPID = this.shooterWheel.getPIDController();
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
            this.driveL1.setInverted(true);
            this.driveR1.setInverted(false);
            
            this.driveL2.follow(this.driveL1, false);
            this.driveR2.follow(this.driveR1, false);

            this.driveL1.setSmartCurrentLimit(60, 10); 
            this.driveL2.setSmartCurrentLimit(60, 10);
            this.driveR1.setSmartCurrentLimit(60, 10);  
            this.driveR2.setSmartCurrentLimit(60, 10);

            this.intakeArm.setInverted(true);
            this.intakeBar.setInverted(false);

            this.stager0.setInverted(true);
            this.stager1.setInverted(true);
            this.stager2.setInverted(true);
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



    /* Intake Motors */

    public void setIntakeArm(double output) {
        this.intakeArm.set(output);
    }

    public void setIntakeBar(double output) {
        this.intakeBar.set(ControlMode.PercentOutput, output);
    }

    public void setIntakeBarRampRate(double rampRateSecondsToFull) {
        this.intakeBar.configClosedloopRamp(rampRateSecondsToFull);
    }

    public CANEncoder getIntakeArmEncoder() {
        return this.intakeArm.getEncoder();
    }


    /* Stager Motors */

    public void setStager0(double output) {
        this.stager0.set(ControlMode.PercentOutput, output);
    }

    public void setStager1(double output) {
        this.stager1.set(ControlMode.PercentOutput, output);
    }

    public void setStager2(double output) {
        this.stager2.set(ControlMode.PercentOutput, output);
    }


    /* Shooter Motors */

    public void setShooterTurret(double output) {
        this.shooterTurret.set(output);
    }

    public void setShooterWheel(double output) {
        this.shooterWheel.set(output);
    }

    public void setShooterWheelRPM(double rpm) {
        this.shooterWheelPID.setReference(rpm, ControlType.kVelocity);
    }

    public CANEncoder getShooterTurretEncoder() {
        return this.shooterTurret.getEncoder();
    }

    public CANEncoder getShooterWheelEncoder() {
        return this.shooterWheel.getEncoder();
    }

    /**
     * Stops all motors
     */
    public void stopAll() {
        setDriveLeft(0);
        setDriveRight(0);
    }

}
