package frc.io;

import org.ejml.dense.row.linsol.qr.BaseLinearSolverQrp_DDRM;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.mode.AutonMode;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.subsystems.Stager;
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
        SmartDashboard.putNumber("ShooterSetRpm", 0);
    }

    public void updateAll() {
        updateSensorDisplay();
    }

    public void updateSensorDisplay() {
        SensorInput sensorIn = SensorInput.getInstance();

        SmartDashboard.putNumber("Gyro", sensorIn.getGyroAngle());
        
        SmartDashboard.putNumber("DriveL1", sensorIn.getDriveL1Encoder());    
        SmartDashboard.putNumber("DriveR1", sensorIn.getDriveR1Encoder());
        SmartDashboard.putNumber("DriveL1FPS", sensorIn.getDriveL1SpeedFPS());

        SmartDashboard.putNumber("Drive-X-pos", sensorIn.getDriveXPos());
        SmartDashboard.putNumber("Drive-Y-pos", sensorIn.getDriveYPos());

        if (Constants.CURRENT_ROBOT == RobotType.COMPBOT2020) {

            SmartDashboard.putNumber("DriveL2", sensorIn.getDriveL2Encoder());
            SmartDashboard.putNumber("DriveR2", sensorIn.getDriveR2Encoder());

            SmartDashboard.putNumber("IntakeArm", sensorIn.getIntakeArmEncoder());

            boolean[] balls = {
                sensorIn.getStagerSensor0(),
                sensorIn.getStagerSensor1(),
                sensorIn.getStagerSensor2()
            };
            for (int i = 0; i < balls.length; i++) {
                SmartDashboard.putBoolean("balls" + i, balls[i]);
            }

            Stager stager = Stager.getInstance();
            SmartDashboard.putString("stagerState", stager.getState().toString());
        }
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
