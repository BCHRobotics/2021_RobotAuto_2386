// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.auton.drive.DriveToPoint;
import frc.io.Dashboard;
import frc.io.RobotOutput;
import frc.io.SensorInput;
import frc.subsystems.Drive;
import frc.teleop.TeleopControl;


public class Robot extends TimedRobot {

  public RobotOutput robotOutput;
  private SensorInput sensorInput;
  private TeleopControl teleopControl;
  private Dashboard dashboard;

  private Drive drive;

  private boolean pushToDashboard = true;
  public static boolean teleopInitialized = false;

  /** This function is called when the robot boots. */
  @Override
  public void robotInit() {

    if (this.pushToDashboard) {
      Constants.pushValues();
    }

    this.robotOutput = RobotOutput.getInstance();
    this.sensorInput = SensorInput.getInstance();
    this.teleopControl = TeleopControl.getInstance();
    this.dashboard = Dashboard.getInstance();
    this.drive = Drive.getInstance();
    this.sensorInput.reset();
  }

  /** This function is called periodically when the robot is on. */
  @Override
  public void robotPeriodic() {}

  /** This function is called once when autonomus is enabled. */
  @Override
  public void autonomousInit() {
    robotOutput.setDriveRampRate(0.0);
    this.sensorInput.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    this.sensorInput.update();
    this.dashboard.updateAll();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // TODO add auto done check

    robotOutput.setDriveRampRate(0.15);
    this.teleopControl.initialize();
    Robot.teleopInitialized = true;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // TODO add auto done check

    if (!Robot.teleopInitialized) {
      this.teleopInit();
    }
    this.sensorInput.update();
    this.teleopControl.runCycle();
    this.dashboard.updateAll();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    this.robotOutput.stopAll();
    this.teleopControl.disable();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    this.sensorInput.update();
    this.dashboard.updateAll();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    this.sensorInput.reset();
    this.drive.firstCycle();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    this.sensorInput.update();
    this.dashboard.updateAll();
  }
}
