package frc.subsystems;

import frc.io.RobotOutput;
import frc.io.SensorInput;

public class Stager extends Subsystem {
    private static Stager instance;

    private RobotOutput robotOut;
    private SensorInput sensorIn;

    public enum StagerState {
        LOAD, UNLOAD, REVERSE_UNLOAD, LOAD_FULL_SPEED, UNLOAD_FULL_SPEED, DELAY_UNLOAD, DELAY_UNLOAD_AUTO, STOP
    }
    private StagerState currentState = StagerState.STOP;
    private double speed;
    private boolean latch;

    private long delayStart;
    
    private double[] stagerOutput = new double[3];

    public static Stager getInstance() {
        if (instance == null) {
            instance = new Stager();
        }
        return instance;
    }

    private Stager() {
        robotOut = RobotOutput.getInstance();
        sensorIn = SensorInput.getInstance();
        this.firstCycle();
    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void setState(StagerState state) {
        this.currentState = state;
    }

    public StagerState getState() {
        return this.currentState;
    }

    public void unlatch() {
        this.latch = false;
    }

    private void load(double speed) {
        boolean[] balls = sensorIn.getStagerSensors();

        if (balls[3] || balls[4]) {
            latch = true;
        }

        stagerOutput[2] = latch ? 0 : speed;
        stagerOutput[1] = (balls[2] && balls[3]) ? 0 : speed * 0.6;
        stagerOutput[0] = (balls[1] && balls[2] && balls[3]) ? 0 : speed * 0.6;
    }

    private void unload(double speed) {
        for (int i = 0; i < stagerOutput.length; i++) {
            stagerOutput[i] = speed;
        }
    }

    private void delayedUnload(long delay) {
        if (delayStart + (delay*3) > System.currentTimeMillis()) {
            stagerOutput[0] = 1;
            stagerOutput[1] = 1;
            stagerOutput[2] = 1;
        } else if (delayStart + (delay*2) > System.currentTimeMillis()) {
            stagerOutput[0] = 0;
            stagerOutput[1] = 1;
            stagerOutput[2] = 1;
        } else if (delayStart + (delay*1) > System.currentTimeMillis()) {
            stagerOutput[0] = 0;
            stagerOutput[1] = 0;
            stagerOutput[2] = 1;
        } else {
            currentState = StagerState.STOP;
        }
    }

    private void reverseUnload(double speed) {
        for (int i = 0; i < stagerOutput.length; i++) {
            stagerOutput[i] = -speed;
        }
    }

    @Override
    public void firstCycle() {

    }

    @Override
    public void calculate() {
        
        if (currentState == StagerState.STOP) {
            robotOut.setStager0(0);
            robotOut.setStager1(0);
            robotOut.setStager2(0);
        } else if (currentState == StagerState.LOAD) {
            load(speed);
        } else if (currentState == StagerState.UNLOAD) {
            unload(speed);
        } else if (currentState == StagerState.LOAD_FULL_SPEED) {
            load(1);
        } else if (currentState == StagerState.UNLOAD_FULL_SPEED) {
            unload(1);
        } else if (currentState == StagerState.DELAY_UNLOAD) {
            delayedUnload(500);
        } else if (currentState == StagerState.DELAY_UNLOAD_AUTO) {
            delayedUnload(2000);
        } else if (currentState == StagerState.REVERSE_UNLOAD) {
            reverseUnload(speed);
        }

        if (currentState != StagerState.DELAY_UNLOAD && currentState != StagerState.DELAY_UNLOAD_AUTO) {
            delayStart = System.currentTimeMillis();
        }

        robotOut.setStager0(stagerOutput[0]);
        robotOut.setStager0(stagerOutput[1]);
        robotOut.setStager0(stagerOutput[2]);
    }

    @Override
    public void disable() {
        robotOut.setStager0(0);
        robotOut.setStager1(0);
        robotOut.setStager2(0);
    }
}
