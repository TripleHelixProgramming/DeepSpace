/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.logger.HelixEvents;
import frc.logger.HelixLogger;
import frc.models.BobTalonSRX;
import frc.models.LeaderBobTalonSRX;
import frc.robot.RobotMap;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {

  private static Drivetrain INSTANCE = new Drivetrain();

  /**
   * @return the singleton instance of the Drivetrain subsystem
   */
  public static Drivetrain getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Drivetrain();
    }
    return INSTANCE;
  }

  private static final int MOTION_PROFILE_POSITIONAL_SLOT = 0;
  private static final int MOTION_PROFILE_HEADING_SLOT = 1;
  private static final int VELOCITY_CONTROL_SLOT = 2;

  // Drivetrain geometry constants
  public static final int DT_ENCODER_TICKS_PER_REV = 480;
  // 4 edges * 120 count quadrature encoder
  // units of rate (ticks per encoder revolution)
  public static final int MAX_DRIVESIDE_VELOCITY = 144;
  // adjusted speed from JVN calculator
  // units of velocity (in/s)
  public static final int DT_HALF_TRACK_WIDTH = 13;
  // distance between the robot centerline and the midpoint of the DT contact
  // patches
  // units of length (in)
  public static final double DT_WHEEL_DIA = 4.0;
  // diameter of wheel
  // units of length (in)
  public static final double DT_ENCODER_GEAR_RATIO = 42.0 / 48;
  // gear ratio between encoder shaft and wheel axle
  // unitless
  public static final double ticks_per_100ms = (DT_ENCODER_TICKS_PER_REV
      / (DT_WHEEL_DIA * Math.PI * DT_ENCODER_GEAR_RATIO * 10.0));
  // factor to convert a linear velocity in in/s to units of counts per 100 ms
  // DT_ENCODER_TICKS_PER_REV - units of rate (ticks per encoder revolution)
  // DT_WHEEL_DIA - diameter of wheel - in
  // DT_ENCODER_GEAR_RATIO - gear ratio between encoder shaft and wheel axle -
  // unitless
  // ticks_per_100ms - factor - (counts * s) / (in * 100 ms)

  private LeaderBobTalonSRX left = new LeaderBobTalonSRX(RobotMap.LEFT_MASTER_ID,
      new BobTalonSRX(RobotMap.LEFT_SLAVE_1_ID), new BobTalonSRX(RobotMap.LEFT_SLAVE_2_ID));
  private LeaderBobTalonSRX right = new LeaderBobTalonSRX(RobotMap.RIGHT_MASTER_ID,
      new BobTalonSRX(RobotMap.RIGHT_SLAVE_1_ID), new BobTalonSRX(RobotMap.RIGHT_SLAVE_2_ID));

  private Drivetrain() {
    setPIDFValues();
    setBrakeMode(NeutralMode.Brake);
    setupSensors();
    setupLogs();

    left.setSensorPhase(false);
    right.setSensorPhase(false);
    left.setInverted(true);
    right.setInverted(false);
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new TestDrive());
    // setDefaultCommand(JoystickDriveFactory.createJoystickDrive());
  }

  /**
   * @param leftVelocity  the velocity the left of the drivetrain should be moving
   *                      at
   * @param rightVelocity the velocity the right of the drivetrain should be
   *                      moving at
   */
  public void drive(double leftVelocity
  , double rightVelocity) {
    left.set(ControlMode.PercentOutput, leftVelocity);
    right.set(ControlMode.PercentOutput, rightVelocity);
  }

  private void setPIDFValues() {
    left.configPIDF(MOTION_PROFILE_POSITIONAL_SLOT, 1, 0, 0, 1.5);
    left.configPIDF(MOTION_PROFILE_HEADING_SLOT, 0, 0, 0, 0);
    left.configPIDF(VELOCITY_CONTROL_SLOT, 0, 0, 0, 0);

    right.configPIDF(MOTION_PROFILE_POSITIONAL_SLOT, 1, 0, 0, 1.5);
    right.configPIDF(MOTION_PROFILE_HEADING_SLOT, 2, 0, 35, 0);
    right.configPIDF(VELOCITY_CONTROL_SLOT, 0, 0, 0, 0);
  }

  private void setupSensors() {
    // Set left pid sensor to it's quad encoder
    left.configPrimaryFeedbackDevice(FeedbackDevice.QuadEncoder);
    left.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);

    // Set the right talon to look at the left's selected sensor (quad encoder) for it's remote sensor 0
    right.configRemoteSensor0(left.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor);
    // Set the right talon to look at the pidgeon plugged into the right slave for it's remote sensor 1
    right.configRemoteSensor1(RobotMap.RIGHT_SLAVE_2_ID, RemoteSensorSource.GadgeteerPigeon_Yaw);

    // Configure the sum PID loop to look at remote sensor 0 and it's own quad encoder
    right.configSensorSum(FeedbackDevice.RemoteSensor0, FeedbackDevice.QuadEncoder);
    // right.configSensorDif(FeedbackDevice.RemoteSensor0, FeedbackDevice.QuadEncoder);
    // Set the primary PID loop to be the sum loop
    right.configPrimaryFeedbackDevice(FeedbackDevice.SensorSum, 0.5);
    // Set the secondary PID loop to be the pidgeon
    right.configSecondaryFeedbackDevice(FeedbackDevice.RemoteSensor1, (3600.0 / 8192.0));
  }

  private void setBrakeMode(NeutralMode neutralMode) {
    left.setNeutralMode(neutralMode);
    right.setNeutralMode(neutralMode);
  }

  private void setupLogs() {
    HelixLogger.getInstance().addSource("LEFT_MASTER_VOLTAGE", () -> Double.toString(left.getMotorOutputVoltage()));
    HelixLogger.getInstance().addSource("LEFT_VELOCITY", () -> Double.toString(left.getSelectedSensorVelocity()));
  }

  /**
   * @return the left master talon
   */
  public LeaderBobTalonSRX getLeft() {
    return left;
  }

  /**
   * @return the right master talon
   */
  public LeaderBobTalonSRX getRight() {
    return right;
  }

  /**
   * @return the average distance of both sides of the drivetrain
   */
  public double getDistance() {
    
    double distance = right.getPrimarySensorPosition();
        HelixEvents.getInstance().addEvent("DRIVETRAIN", "Current position: " + distance);
        return distance;
  }

  public void resetEncoders() {
    left.getSensorCollection().setQuadraturePosition(0, 0);
    right.getSensorCollection().setQuadraturePosition(0, 0);
  }
}
