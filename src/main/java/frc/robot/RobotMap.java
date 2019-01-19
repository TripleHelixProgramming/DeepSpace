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

  //DRIVETRAIN
  public static int LEFT_MASTER_ID = 14;
  public static int LEFT_SLAVE_1_ID = 12;
  public static int LEFT_SLAVE_2_ID = 10;
  public static int RIGHT_MASTER_ID = 15;
  public static int RIGHT_SLAVE_1_ID = 13;
  public static int RIGHT_SLAVE_2_ID = 11;
  
  //HATCHGRABBER
  public static final int HATCH_GRAB = 2;
  public static final int HATCH_RELEASE = 3;
  
  //LIMIT SWITCHES
  public static final int HATCH_LIMIT_CHANNEL = 2;

}
