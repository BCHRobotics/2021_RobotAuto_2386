package frc.io;

import frc.util.XboxController;
import frc.util.XboxController.Axis;
import frc.util.XboxController.Button;
import frc.util.XboxController.Side;

public class DriverInput {
    
    private static DriverInput instance;

	private XboxController driver;
	private XboxController operator;

	// Creates boolean variables that stores if a certain step/mode was pressed
	private boolean autonIncreaseStepWasPressed = false;
	private boolean autonDecreaseStepWasPressed = false;

	private boolean autonIncreaseModeWasPressed = false;
	private boolean autonDecreaseModeWasPressed = false;

	private boolean autonIncreaseMode10WasPressed = false;
	private boolean autonDecreaseMode10WasPressed = false;

	private DriverInput() {
		this.driver = new XboxController(0);
		this.operator = new XboxController(1);
	}

	public static DriverInput getInstance() {
		if (instance == null) {
			instance = new DriverInput();
		}
		return instance;
	}

    public XboxController getDriverController() {
        return driver;
    }

    public XboxController getOperatorController() {
        return operator;
    }

    // ********************************
	// AUTO SELECTION CONTROLS
	// ********************************

	public boolean getResumeAutoButton() {
		return driver.getButton(Button.Y);
	}

	public boolean getDriverAutoOverrideButtons() {
		return this.driver.getButton(Button.Y);
	}

	public boolean getOperatorAutoOverrideButtons() {
		return this.operator.getButton(Button.A);
	}

	public boolean getAutonSetDelayButton() {
		return false;//this.driver.getRightTrigger() > 0.2;
	}

	public double getAutonDelayStick() {
		return this.driver.getJoystick(Side.LEFT, Axis.Y);
	}

	public boolean getAutonStepIncrease() {
		// only returns true on rising edge
		boolean result = this.driver.getButton(Button.RB) && !this.autonIncreaseStepWasPressed;
		this.autonIncreaseStepWasPressed = this.driver.getButton(Button.RB);
		return result;

	}

	public boolean getAutonStepDecrease() {
		// only returns true on rising edge
		boolean result = this.driver.getButton(Button.LB) && !this.autonDecreaseStepWasPressed;
		this.autonDecreaseStepWasPressed = this.driver.getButton(Button.LB);
		return result;

	}

	public boolean getAutonModeIncrease() {
		// only returns true on rising edge
		boolean result = this.driver.getButton(Button.X) && !this.autonIncreaseModeWasPressed;
		this.autonIncreaseModeWasPressed = this.driver.getButton(Button.X);
		return result;

	}

	public boolean getAutonModeDecrease() {
		// only returns true on rising edge
		boolean result = this.driver.getButton(Button.A) && !this.autonDecreaseModeWasPressed;
		this.autonDecreaseModeWasPressed = this.driver.getButton(Button.A);
		return result;

	}

	public boolean getAutonModeIncreaseBy10() {
		// only returns true on rising edge
		boolean result = this.driver.getButton(Button.Y) && !this.autonIncreaseMode10WasPressed;
		this.autonIncreaseMode10WasPressed = this.driver.getButton(Button.Y);
		return result;

	}

	public boolean getAutonModeDecreaseBy10() {
		// only returns true on rising edge
		boolean result = this.driver.getButton(Button.X) && !this.autonDecreaseMode10WasPressed;
		this.autonDecreaseMode10WasPressed = this.driver.getButton(Button.X);
		return result;

	}

}
