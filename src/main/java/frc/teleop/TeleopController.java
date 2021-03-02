package frc.teleop;

import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.subsystems.Drive;
import frc.util.XboxController;
import frc.util.XboxController.Axis;
import frc.util.XboxController.Side;

public class TeleopController extends TeleopComponent {

    private static TeleopController instance;

    private XboxController driverController;
    private XboxController operatorController;

    private SensorInput sensorInput;
    private RobotOutput robotOutput;
    private Drive drive;
    
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
    }

    @Override
    public void firstCycle() {
        this.drive.firstCycle();
    }

    @Override
    public void calculate() {

        drive.setOutput(
            driverController.getJoystick(Side.LEFT, Axis.Y), 
            driverController.getJoystick(Side.RIGHT, Axis.X)
        );

        this.drive.calculate();
    }

    @Override
    public void disable() {
        this.drive.disable();
    }
    
}
