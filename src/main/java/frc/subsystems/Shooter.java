package frc.subsystems;

import frc.io.RobotOutput;
import frc.io.SensorInput;

public class Shooter extends Subsystem {
    private static Shooter instance;

    private RobotOutput robotOut;
    private SensorInput sensorIn;

    private double turretSpeed;
    private double wheelSpeed;

    private static final int minTurret = -75;
    private static final int minSafeTurret = -45;
    private static final int minVerySafeTurret = -15;
    private static final int maxTurret = 190;
    private static final int maxSafeTurret = 160;
    private static final int maxVerySafeTurret = 130;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        robotOut = RobotOutput.getInstance();
        sensorIn = SensorInput.getInstance();
        this.firstCycle();
    }

    @Override
    public void firstCycle() {

    }

    public void setTurretSpeed(double speed) {
        double position = sensorIn.getShooterTurretEncoder();

        if ((position <= minTurret && speed < 0) || (position >= maxTurret && speed > 0)) {
            turretSpeed = 0;
        } else if ((position <= minSafeTurret && speed < 0) || (position >= maxSafeTurret && speed > 0)) {
            turretSpeed = speed * 0.1;
        } else if ((position <= minVerySafeTurret && speed < 0) || (position >= maxVerySafeTurret && speed > 0)) {
            turretSpeed = speed * 0.2;
        } else {
            turretSpeed = speed;
        }
    }

    public void setWheelSpeed(double speed) {
        this.wheelSpeed = speed;
    }

    @Override
    public void calculate() {
        robotOut.setShooterTurret(turretSpeed);
        robotOut.setShooterWheel(wheelSpeed);
    }

    @Override
    public void disable() {
        robotOut.setShooterTurret(0);
    }
}
