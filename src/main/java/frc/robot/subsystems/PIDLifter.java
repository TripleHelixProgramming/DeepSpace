/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.*;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team2363.logger.HelixLogger;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.PIDLifter;
import frc.robot.RobotMap;

public class PIDLifter extends Subsystem {

  PowerDistributionPanel pdp = new PowerDistributionPanel();

  private TalonSRX lifterMaster = new TalonSRX(RobotMap.LIFTER_LEFT_ID);
  private TalonSRX lifterSlave = new TalonSRX(RobotMap.LIFTER_RIGHT_ID);

  public static int LIFTER_ACCELERATION = 5000;
  public static int LIFTER_CRUISE = 1918;
  // public static int LIFTER_CRUISE = 700;


  private static PIDLifter INSTANCE = new PIDLifter();

  public enum LiftPos {
    BURST(786), EXTEND(24000);

    private final double pos;

    private LiftPos(double pos) {
      this.pos = pos;
    }

    public double getPos() {
      return pos;
    }
  }

  public static PIDLifter getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new PIDLifter();
    }
    return INSTANCE;

  }

  public PIDLifter() {
    setupLogs();
    encoderReset();

    // Configure motors to factory default.
    lifterSlave.configFactoryDefault();
    lifterMaster.configFactoryDefault();

    
    // lifterSlave.

    // Set current limiting
    lifterMaster.configContinuousCurrentLimit(40, 0);
    lifterMaster.configPeakCurrentLimit(60, 0);
    lifterMaster.configPeakCurrentDuration(100, 0);
    lifterMaster.enableCurrentLimit(true);

    //lifterMaster.configOpenloopRamp(0, 0);
    // lifterMaster.configClosedloopRamp(.1);
    lifterMaster.setNeutralMode(NeutralMode.Brake);

    // Configure sensor inputs
    lifterMaster.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    lifterMaster.overrideLimitSwitchesEnable(true);
    lifterMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // lifterMaster.configFeedbackNotContinuous(true, RobotMap.CTRE_TIMEOUT_INIT);

    lifterMaster.setSensorPhase(false);  // + motor power must have increasing sensor values
    lifterMaster.setInverted(false);

    lifterSlave.setInverted(InvertType.OpposeMaster);
    lifterSlave.setNeutralMode(NeutralMode.Brake);
    lifterSlave.configOpenloopRamp(0, 0);
    lifterSlave.follow(lifterMaster);

    // PID Settings - PB
    // lifterMaster.config_kF(0, 1.0, RobotMap.CTRE_TIMEOUT_INIT);
    // lifterMaster.config_kF(0, 30, RobotMap.CTRE_TIMEOUT_INIT);
    // lifterMaster.config_kP(0, .1, RobotMap.CTRE_TIMEOUT_INIT);
    lifterMaster.config_kF(0, .32, RobotMap.CTRE_TIMEOUT_INIT);
    lifterMaster.config_kP(0, 1, RobotMap.CTRE_TIMEOUT_INIT);
    lifterMaster.config_kI(0, 0.0, RobotMap.CTRE_TIMEOUT_INIT);
    lifterMaster.config_kD(0, 0, RobotMap.CTRE_TIMEOUT_INIT);
    lifterMaster.configAllowableClosedloopError(0, 0, RobotMap.CTRE_TIMEOUT_INIT);

    //  5000 ACC Elevator   700 Cruise Elevator
    lifterMaster.configMotionAcceleration(LIFTER_ACCELERATION,RobotMap.CTRE_TIMEOUT_INIT);
    lifterMaster.configMotionCruiseVelocity(LIFTER_CRUISE, RobotMap.CTRE_TIMEOUT_INIT);
  }

  public double getPosition() {
    
    return lifterMaster.getSensorCollection().getQuadraturePosition();

  }

  public double getVelocity() {
    return lifterMaster.getSensorCollection().getQuadratureVelocity();
  }

  public void encoderReset() {
    lifterMaster.getSensorCollection().setQuadraturePosition(0, 0);
  }

  public void goTo(LiftPos pos) {
    goTo(pos.getPos());
  }

  public void goTo(double pos) {
    lifterMaster.set(ControlMode.MotionMagic, pos);
  }

  public boolean isLimitSwitchTriggered() {
    return lifterMaster.getSensorCollection().isRevLimitSwitchClosed();
  }

  // Used by DPAD commands to be check if movement command finished.
  public boolean isBurstDone() {
    return (Math.abs(getPosition() - LiftPos.BURST.getPos()) <= 20);
  }

  // Used by DPAD commands to be check if movement command finished.
  public boolean isExtendDone() {
    return (Math.abs(getPosition() - LiftPos.EXTEND.getPos()) <= 2);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lifter Encoder Pos", getPosition());
    SmartDashboard.putNumber("Burst Target", LiftPos.BURST.getPos());
    SmartDashboard.putNumber("Extend Target", LiftPos.EXTEND.getPos());
    SmartDashboard.putBoolean("LimitSwitch", isLimitSwitchTriggered());
    SmartDashboard.putNumber("Error", lifterMaster.getClosedLoopError());
  }

  private void setupLogs() {
    HelixLogger.getInstance().addDoubleSource("LIFTER MASTER", lifterMaster::getOutputCurrent);
    HelixLogger.getInstance().addDoubleSource("LIFTER SLAVE", ()-> pdp.getCurrent(RobotMap.LIFTER_RIGHT_PDP));
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
