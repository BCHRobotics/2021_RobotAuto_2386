package frc.teleop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.subsystems.Drive;
import frc.subsystems.Intake;
import frc.subsystems.Shooter;
import frc.subsystems.Stager;
import frc.subsystems.Stager.StagerState;
import frc.util.XboxController;
import frc.util.XboxController.Axis;
import frc.util.XboxController.Button;
import frc.util.XboxController.Side;

public class TeleopController extends TeleopComponent {

    private static TeleopController instance;

    private XboxController driverController;
    private XboxController operatorController;

    private SensorInput sensorInput;
    private RobotOutput robotOutput;
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Stager stager;

    private enum OperatorMode {
        CLIMB, DRIVE
    }
    private OperatorMode operatorMode = OperatorMode.DRIVE;
    private boolean deBounce = false;
    private long deBounceEnd;

    public static TeleopController getInstance() {
		if (instance == null) {
			instance = new TeleopController();
		}
		return instance;
	}

    private TeleopController() {
        this.driverController = new XboxController(0);
        this.operatorController = new XboxController(1);

        this.sensorInput = SensorInput.getInstance();
        this.robotOutput = RobotOutput.getInstance();
        this.drive = Drive.getInstance();
        this.shooter = Shooter.getInstance();
        this.intake = Intake.getInstance();
        this.stager = Stager.getInstance();
    }

    @Override
    public void firstCycle() {
        this.drive.firstCycle();
    }

    private void driver() {

        double speed = 0.75;

        if (driverController.getButton(Button.LB)) speed = 0.5;
        if (driverController.getButton(Button.RB)) speed = 1.0;

        drive.setOutput(
            deadzone(driverController.getJoystick(Side.LEFT, Axis.Y), 0.07) * speed, 
            deadzone(driverController.getJoystick(Side.RIGHT, Axis.X), 0.07) * speed
        );
    }

    private void operatorDrive() {

        /* Shooter */
        shooter.setTurretSpeed(deadzone(operatorController.getJoystick(Side.LEFT, Axis.X), 0.2));

        if (-operatorController.getJoystick(Side.LEFT, Axis.Y) >= 0.3) {
            shooter.setWheelSpeed(-operatorController.getJoystick(Side.LEFT, Axis.Y));
        } else {
            shooter.setWheelSpeed(0);
        }

        /* Intake */
        double intakeSpeed = operatorController.getTrigger(Side.LEFT) - operatorController.getTrigger(Side.RIGHT);
        intake.setIntakeSpeed(intakeSpeed);
        if (intakeSpeed != 0) {
            stager.setState(StagerState.LOAD);
        }

        if (operatorController.getButton(Button.RB)) {
            intake.setArmSpeed(0.1);
        } else if (operatorController.getButton(Button.LB)) {
            intake.setArmSpeed(-0.25);
        } else {
            intake.setArmSpeed(0);
        }

        /* Stager */
        if (operatorController.getButton(Button.X)) {
            stager.setSpeed(1);
            stager.setState(StagerState.UNLOAD_FULL_SPEED);
        }

        if (!operatorController.getButton(Button.Y) && intakeSpeed != 0) {
            stager.setSpeed(0.47);
            stager.setState(StagerState.LOAD);
        } else if (operatorController.getButton(Button.X)) {
            stager.setSpeed(1);
            stager.setState(StagerState.DELAY_UNLOAD);
        } else {
            stager.setState(StagerState.STOP);
        }
    }

    private void operatorClimb() {

    }

    @Override
    public void calculate() {
        
        driver();

        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {
            if (operatorMode == OperatorMode.DRIVE) {
                operatorDrive();
            } else if (operatorMode == OperatorMode.CLIMB) {
                operatorClimb();
            }

            /** Debounce checker control for switching operator modes */
            if ((operatorController.getButton(Button.START) || operatorController.getButton(Button.BACK)) && !deBounce) {
                deBounce = true;
                deBounceEnd = System.currentTimeMillis() + 250;
                operatorController.setRumble(true);

                operatorMode = operatorMode == OperatorMode.DRIVE ? OperatorMode.CLIMB : OperatorMode.DRIVE;
            }

            if (System.currentTimeMillis() > deBounceEnd) {
                deBounce = false;
                operatorController.setRumble(false);
            }

            this.shooter.calculate();
            this.intake.calculate();
            this.stager.calculate();
        }

        /* Calculates */
        this.drive.calculate(); 
    }

    private static double deadzone(double input, double deadzone) {
        return (input < deadzone && input > -deadzone) ? 0 : input;
    }

    @Override
    public void disable() {
        this.drive.disable();
    }
    
}
