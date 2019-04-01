/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team2363.logger.HelixEvents;
import com.team2363.logger.HelixLogger;
import com.team319.follower.FollowsArc;
import com.team319.models.BobTalonSRX;
import com.team319.models.BobVictorSPX;
import com.team319.models.LeaderBobTalonSRX;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.drivetrain.DustinDrive;
import frc.robot.commands.drivetrain.JoshDrive;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Drivetrain extends Subsystem implements FollowsArc {

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

  // Programming Robot =  All Talons
  // private BobTalonSRX rightSlave1 = new BobTalonSRX(RobotMap.RIGHT_SLAVE_1_ID);
  // private BobTalonSRX rightSlave2 = new BobTalonSRX(RobotMap.RIGHT_SLAVE_2_ID);
  // private BobTalonSRX leftSlave1 = new BobTalonSRX(RobotMap.LEFT_SLAVE_1_ID);
  // private BobTalonSRX leftSlave2 = new BobTalonSRX(RobotMap.LEFT_SLAVE_2_ID);

  //  Competition & Practice Bot  Talon Masters with Victors as Slaves.
  private BobVictorSPX rightSlave1 = new BobVictorSPX(RobotMap.RIGHT_SLAVE_1_ID);
  private BobVictorSPX rightSlave2 = new BobVictorSPX(RobotMap.RIGHT_SLAVE_2_ID);
  private BobVictorSPX leftSlave1 = new BobVictorSPX(RobotMap.LEFT_SLAVE_1_ID);
  private BobVictorSPX leftSlave2 = new BobVictorSPX(RobotMap.LEFT_SLAVE_2_ID);

  private LeaderBobTalonSRX left = new LeaderBobTalonSRX(RobotMap.LEFT_MASTER_ID,
      leftSlave1, leftSlave2);
  private LeaderBobTalonSRX right = new LeaderBobTalonSRX(RobotMap.RIGHT_MASTER_ID,
      rightSlave1, rightSlave2);

  private TalonSRX cargoGrabberLeft = new TalonSRX(RobotMap.CARGO_LEFT_ID);
    
  PowerDistributionPanel pdp = new PowerDistributionPanel();
  private PigeonIMU pigeon = new PigeonIMU(cargoGrabberLeft);

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
    setDefaultCommand(new DustinDrive());
  }

  public void tankDrive(double leftVelocity, double rightVelocity) {
    left.set(PercentOutput, leftVelocity);
    right.set(PercentOutput, rightVelocity);
  }

  public void arcadeDrive(double throttle, double turn, boolean squaredInputs) {
    if (squaredInputs) {
      // square the inputs (while preserving the sign) to increase fine control
      // while permitting full power
      throttle = Math.abs(throttle) * throttle;
      turn = Math.abs(turn) * turn;
    }

    left.set(PercentOutput, throttle - turn);
    right.set(PercentOutput, throttle + turn);
	}

  private void setPIDFValues() {
    left.configPIDF(MOTION_PROFILE_POSITIONAL_SLOT, 7.17, 0, 0, 2);
    left.configPIDF(MOTION_PROFILE_HEADING_SLOT, 0, 0, 0, 0);
    left.configPIDF(VELOCITY_CONTROL_SLOT, 0, 0, 0, 0);

    right.configPIDF(MOTION_PROFILE_POSITIONAL_SLOT, 7.16, 0, 0, 2);
    right.configPIDF(MOTION_PROFILE_HEADING_SLOT, 2, 0, 55, 0);
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

    // HelixLogger.getInstance().addDoubleSource("TOTAL CURRENT", pdp::getTotalCurrent);
    HelixLogger.getInstance().addDoubleSource("DT LM Current", left::getOutputCurrent);
    HelixLogger.getInstance().addDoubleSource("DT RM Current", right::getOutputCurrent);

    //  This logging format should work for Talons OR Victor SLAVES.
    HelixLogger.getInstance().addDoubleSource("DT LS1 Current", () -> pdp.getCurrent(RobotMap.LEFT_SLAVE_1_PDP));
    HelixLogger.getInstance().addDoubleSource("DT LS2 Current", () -> pdp.getCurrent(RobotMap.LEFT_SLAVE_2_PDP));
    HelixLogger.getInstance().addDoubleSource("DT RS1 Current", () -> pdp.getCurrent(RobotMap.RIGHT_SLAVE_1_PDP));
    HelixLogger.getInstance().addDoubleSource("DT RS2 Current", () -> pdp.getCurrent(RobotMap.RIGHT_SLAVE_1_PDP));

    HelixLogger.getInstance().addDoubleSource("PIGEON HEADING", () -> this.getYaw());

    HelixLogger.getInstance().addDoubleSource("DRIVETRAIN LEFT Voltage", left::getMotorOutputVoltage);
    HelixLogger.getInstance().addIntegerSource("DRIVETRAIN LEFT Velocity", this::getLeftVelocity);
    HelixLogger.getInstance().addDoubleSource("DRIVETRAIN RIGHT Voltage", right::getMotorOutputVoltage);
    HelixLogger.getInstance().addIntegerSource("DRIVETRAIN RIGHT Velocity", this::getRightVelocity);
  }

  @Override
  public BobTalonSRX getLeft() {
    return left;
  }

  @Override
  public BobTalonSRX getRight() {
    return right;
  }

  @Override
  public double getDistance() {
    
    double distance = right.getPrimarySensorPosition();
        HelixEvents.getInstance().addEvent("DRIVETRAIN", "Current position: " + distance);
        return distance;
  }

  public void resetEncoders() {
    left.getSensorCollection().setQuadraturePosition(0, 0);
    right.getSensorCollection().setQuadraturePosition(0, 0);
  }

  @Override
  public Subsystem getRequiredSubsystem() {
    return this;
  }

  public void resetHeading() {
    pigeon.setYaw(0.0);
  }

  public double getYaw() {
    double [] yaw = {0, 0, 0};
    pigeon.getYawPitchRoll(yaw);
    return yaw[0];
  }

  public int getLeftVelocity() {
    try {
      return left.getSelectedSensorVelocity();
    } catch(Exception e) {
      SmartDashboard.putString("Exception", e.getMessage());
      return 100000000;
    } catch(Throwable t) {
      SmartDashboard.putString("Exception", t.getMessage());
      return 1999999999;
    }
  }

  public int getRightVelocity() {
    return right.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pigeon Yaw", getYaw());
  }
}
