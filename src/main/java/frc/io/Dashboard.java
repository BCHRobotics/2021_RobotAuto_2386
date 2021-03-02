package frc.io;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.util.PIDConstants;

public class Dashboard {
    private static Dashboard instance;

    public static Dashboard getInstance() {
        if (instance == null) {
            instance = new Dashboard();
        }
        return instance;
    }

    public Dashboard() {
        SmartDashboard.putNumber("Path Turn P", Constants.PATH_TURN_P);
    }

    public void updateAll() {
        updateSensorDisplay();
    }

    public void updateSensorDisplay() {
        SensorInput sensorInput = SensorInput.getInstance();

        SmartDashboard.putNumber("Gyro", sensorInput.getGyroAngle());
        
        SmartDashboard.putNumber("DriveL1", sensorInput.getDriveL1Encoder());
        SmartDashboard.putNumber("DriveL2", sensorInput.getDriveL2Encoder());
        SmartDashboard.putNumber("DriveR1", sensorInput.getDriveR1Encoder());
        SmartDashboard.putNumber("DriveR2", sensorInput.getDriveR2Encoder());

        SmartDashboard.putNumber("DriveL1FPS", sensorInput.getDriveL1SpeedFPS());
    }

    // Get the PID Constants
	public PIDConstants getPIDConstants(String name, PIDConstants constants) {
		double p = SmartDashboard.getNumber(name + "-P", constants.p);
		double i = SmartDashboard.getNumber(name + "-I", constants.i);
		double d = SmartDashboard.getNumber(name + "-D", constants.d);
		double ff = SmartDashboard.getNumber(name + "-FF", constants.ff);
		double eps = SmartDashboard.getNumber(name + "-EPS", constants.eps);
		return new PIDConstants(p, i, d, ff, eps);
	}

	// Put the PID Constants on the dashboard
	public void putPIDConstants(String name, PIDConstants constants) {
        SmartDashboard.putNumber(name + "-P", constants.p);
        SmartDashboard.putNumber(name + "-I", constants.i);
		SmartDashboard.putNumber(name + "-D", constants.d);
		SmartDashboard.putNumber(name + "-FF", constants.ff);
		SmartDashboard.putNumber(name + "-EPS", constants.eps);
	}

    // Get the PID Turn
	public double getPathTurnP() {
		return SmartDashboard.getNumber("Path Turn P", Constants.PATH_TURN_P);
	}
}
