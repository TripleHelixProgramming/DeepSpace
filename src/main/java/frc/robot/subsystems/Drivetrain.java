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
import frc.models.BobTalonSRX;
import frc.models.LeaderBobTalonSRX;
import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.JoystickDrive;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem {

  private static Drivetrain INSTANCE;

   /**
   * @return the singleton instance of the Drivetrain subsystem
   */
  public static Drivetrain getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Drivetrain();
    }
    return INSTANCE;
  }

  //Drivetrain geometry constants
	public static final int DT_ENCODER_TICKS_PER_REV = 480;
  //4 edges * 120 count quadrature encoder
  //units of rate (ticks per encoder revolution)
public static final int MAX_DRIVESIDE_VELOCITY = 144;
  //adjusted speed from JVN calculator
  //units of velocity (in/s)
public static final int DT_HALF_TRACK_WIDTH = 13;
  //distance between the robot centerline and the midpoint of the DT contact patches
  //units of length (in)
public static final double DT_WHEEL_DIA = 4.0;
  //diameter of wheel
  //units of length (in)
public static final double DT_ENCODER_GEAR_RATIO = 42.0 / 48;
  //gear ratio between encoder shaft and wheel axle
  //unitless
public static final double ticks_per_100ms = (DT_ENCODER_TICKS_PER_REV / (DT_WHEEL_DIA * Math.PI * DT_ENCODER_GEAR_RATIO * 10.0));
  //factor to convert a linear velocity in in/s to units of counts per 100 ms
  //DT_ENCODER_TICKS_PER_REV - units of rate (ticks per encoder revolution)
  //DT_WHEEL_DIA - diameter of wheel - in
  //DT_ENCODER_GEAR_RATIO - gear ratio between encoder shaft and wheel axle - unitless
  //ticks_per_100ms - factor - (counts * s) / (in * 100 ms)

  private LeaderBobTalonSRX left = new LeaderBobTalonSRX(RobotMap.LEFT_MASTER_ID, 
      new BobTalonSRX(RobotMap.LEFT_SLAVE_1_ID), 
      new BobTalonSRX(RobotMap.LEFT_SLAVE_2_ID));
  private LeaderBobTalonSRX right = new LeaderBobTalonSRX(RobotMap.RIGHT_MASTER_ID, 
      new BobTalonSRX(RobotMap.RIGHT_SLAVE_1_ID), 
      new BobTalonSRX(RobotMap.RIGHT_SLAVE_2_ID));

  private Drivetrain() { 
    setPIDFValues();
    setNeutralMode(NeutralMode.Brake);
    setupSensors();
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickDrive());
  }

  /**
   * @param leftVelocity the velocity the left of the drivetrain should be moving at
   * @param rightVelocity the velocity the right of the drivetrain should be moving at
   */
  public void drive(double leftVelocity, double rightVelocity) {
		left.set(ControlMode.Velocity, leftVelocity);
		right.set(ControlMode.Velocity, rightVelocity);
	}

  private void setPIDFValues() {
    left.configPIDF(0, 0, 0, 0, 0); // Motion profiling distance slot
    left.configPIDF(1, 0, 0, 0, 0); // Motion profiling turning slot
    left.configPIDF(2, 0, 0, 0, 0); // Velocity slot

    right.configPIDF(0, 0, 0, 0, 0); // Motion profiling distance slot
    right.configPIDF(1, 0, 0, 0, 0); // Motion profiling turning slot
    right.configPIDF(2, 0, 0, 0, 0); // Velocity slot
  }

  private void setupSensors() {
    right.configRemoteSensor0(left.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor);
		right.configSensorSum(FeedbackDevice.RemoteSensor0, FeedbackDevice.CTRE_MagEncoder_Relative);
    right.configPrimaryFeedbackDevice(FeedbackDevice.SensorSum, 0.5);
    
    right.configRemoteSensor1(1, RemoteSensorSource.GadgeteerPigeon_Yaw);
		right.configSecondaryFeedbackDevice(FeedbackDevice.RemoteSensor1, (3600.0 / 8192.0)); 
		left.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
  }

  private void setNeutralMode(NeutralMode neutralMode) {
		left.setNeutralMode(neutralMode);
		right.setNeutralMode(neutralMode);
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
    return (left.getSensorCollection().getQuadraturePosition()
				+ right.getSensorCollection().getQuadraturePosition()) / 2;
  }
}
