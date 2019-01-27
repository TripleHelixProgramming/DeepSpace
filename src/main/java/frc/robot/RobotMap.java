/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static int LEFT_MASTER_ID = 10;
  public static int LEFT_SLAVE_1_ID = 12;
  public static int LEFT_SLAVE_2_ID = 14;
  public static int RIGHT_MASTER_ID = 11;
  public static int RIGHT_SLAVE_1_ID = 13;
  public static int RIGHT_SLAVE_2_ID = 15;
  
  // Jester Arm 
  public static int WRIST_TALON_ID = 1;
  public static int WRIST_ENCODER_ID = 4;
  public static int SHOULDER_MASTER_TALON_ID = 2;
  public static int SHOULDER_SLAVE_TALON_ID = 3;
  public static int TELESCOPE_TALON_ID = 4;
  public static final int CTRE_TIMEOUT_INIT = 10;
  public static final int CTRE_TIMEOUT_PERIODIC = 0;

  //CargoGrabber Solenoids
  public static int CARGO_EXTEND_SOLENOID = 6;
  public static int CARGO_RETRACT_SOLENOID = 1;

  public static final int CARGO_LEFT_WHEEL = 20;
	public static final int CARGO_RIGHT_WHEEL = 21;
  
  //HATCHGRABBER
  public static final int HATCH_GRAB = 2;
  public static final int HATCH_RELEASE = 3;
  
  //LIMIT SWITCHES
  public static final int HATCH_LIMIT_CHANNEL = 2;
}
