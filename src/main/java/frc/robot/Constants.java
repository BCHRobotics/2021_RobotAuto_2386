package frc.robot;

import frc.io.Dashboard;
import frc.util.PIDConstants;

public class Constants {
    
    public static final boolean USING_DASHBOARD = true;

    public enum RobotType {
        MINIBOT, COMPBOT2020
    }
    public static final RobotType CURRENT_ROBOT = RobotType.MINIBOT;
    public static final double MAXOUTPUT = 1;

    // Intake
    public static final double intakeArmMax = 45;
    public static final double intakeArmMin = 0;

    // Controllers
    public static final double xboxControllerDeadzone = 0.1;

    // PID
    public static final double PATH_TURN_P = 6;

    private static double miniDriveWheelDiameter = 5.78;
    private static double comp2020DriveWheelDiameter = 0;

	private static PIDConstants driveStraightPID = new PIDConstants(3.0, 0.04, 5.0, 0.05);
	private static PIDConstants driveTurnPID = new PIDConstants(0.2, 0.005, 0.6, 1);
	private static PIDConstants driveVelocityPID = new PIDConstants(0.0, 0, 0.0, 1.0 / 14.0, 0);

    private static PIDConstants turretPID = new PIDConstants(0, 0, 0, 0);

    private static Dashboard dashboard = Dashboard.getInstance();

    public static PIDConstants getDriveStraightPID() {
        if (USING_DASHBOARD) {
            return dashboard.getPIDConstants("DRIVE_PID", driveStraightPID);
        } else {
            return driveStraightPID;
        }
    }

    public static PIDConstants getDriveTurnPID() {
        if (USING_DASHBOARD) {
            return dashboard.getPIDConstants("TURN_PID", driveTurnPID);
        } else {
            return driveTurnPID;
        }
    }

    public static PIDConstants getDriveVelocityPID() {
        if (USING_DASHBOARD) {
            return dashboard.getPIDConstants("DRIVE_VELOCITY_PID", driveVelocityPID);
        } else {
            return driveVelocityPID;
        }
    }

    public static PIDConstants getTurretPID() {
        if (USING_DASHBOARD) {
            return dashboard.getPIDConstants("TURRET_PID", turretPID);
        } else {
            return turretPID;
        }
    }

    public static double getDriveWheelDiameter() {
        if (CURRENT_ROBOT == RobotType.COMPBOT2020) {
            return comp2020DriveWheelDiameter;
        } else if (CURRENT_ROBOT == RobotType.MINIBOT) {
            return miniDriveWheelDiameter;
        }
        return 0;
    }

    public static void pushValues() {
        dashboard.putPIDConstants("DRIVE_PID", driveStraightPID);
        dashboard.putPIDConstants("TURN_PID", driveTurnPID);
        dashboard.putPIDConstants("DRIVE_VELOCITY_PID", driveVelocityPID);
        dashboard.putPIDConstants("TURRET_PID", turretPID);
    }
    
}
