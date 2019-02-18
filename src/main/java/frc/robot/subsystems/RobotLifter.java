/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team2363.logger.HelixLogger;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Utility;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.robot_lifter.StopLifter;

public class RobotLifter extends Subsystem {

  PowerDistributionPanel pdp = new PowerDistributionPanel();

  private TalonSRX left = new TalonSRX(RobotMap.LIFTER_LEFT_ID);
  private VictorSPX right = new VictorSPX(RobotMap.LIFTER_RIGHT_ID);

  private static RobotLifter INSTANCE = new RobotLifter();

  public static RobotLifter getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new RobotLifter();
    }
    return INSTANCE;
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new StopLifter());
  }
  
  public RobotLifter(){ 
    // setupLogs();

    left.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		left.overrideLimitSwitchesEnable(true);

    right.setNeutralMode(NeutralMode.Brake);
    left.setNeutralMode(NeutralMode.Brake);

    // Set current limiting
    left.configContinuousCurrentLimit(40, 0);
		left.configPeakCurrentLimit(60, 0);
		left.configPeakCurrentDuration(100, 0);
    left.enableCurrentLimit(true);

  }
  
  @Override
	public void periodic() {
    SmartDashboard.putBoolean("LimitSwitch", isLimitSwitchTriggered());
  }

  public boolean isLimitSwitchTriggered() {
    return left.getSensorCollection().isRevLimitSwitchClosed();
  }

  

  public void setPower(double power){
    left.set(ControlMode.PercentOutput,  power);
    right.set(ControlMode.PercentOutput, -power);
  }

  private void setupLogs() {
    HelixLogger.getInstance().addDoubleSource("LIFTER_LEFT_CURRENT", left::getOutputCurrent);
    HelixLogger.getInstance().addDoubleSource("LIFTER_RIGHT_CURRENT", ()-> pdp.getCurrent(RobotMap.LIFTER_RIGHT_PDP));
   }
}

