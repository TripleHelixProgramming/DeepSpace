/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.camera.CAMERA;
import frc.robot.commands.Auto.FollowArcTesting;
import frc.robot.commands.Auto.PickUpCargo;
import frc.robot.commands.Auto.resetCargoJester;
import frc.robot.commands.cargo_grabber.GrabCargo;
import frc.robot.commands.cargo_grabber.ReleaseCargo;
import frc.robot.commands.cargo_grabber.stopCargoGrabber;
import frc.robot.commands.drivetrain.driveByCamera;
import frc.robot.commands.drivetrain.driveByDocking;
import frc.robot.commands.drivetrain.driveByDockingPID;
import frc.robot.commands.hatch.GrabHatch;
import frc.robot.commands.hatch.ReleaseHatch;
import frc.robot.commands.jester_arm.ToggleArmCommand;
import frc.robot.commands.jester_arm.downCommand;
import frc.robot.commands.jester_arm.unDockArm;
import frc.robot.commands.jester_arm.upCommand;
import frc.robot.commands.robot_lifter.ExtendLifter;
import frc.robot.commands.robot_lifter.burstExtendLifter;
import frc.robot.commands.robot_lifter.reverseLifter;


// import frc.robot.commands.drivetrain.driveByDockingPID;
// import frc.robot.commands.drivetrain.aimByVision;
// import frc.robot.commands.Camera.driveByVision;
// import frc.robot.commands.jester_arm.ToggleArmCommand;
// import frc.robot.commands.jester_arm.ToggleHeightCommand;
// import frc.robot.commands.Camera.driveByAssist;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private static OI INSTANCE;

  /**
   * @return retrieves the singleton instance of the Operator Interface
   */
  public static OI getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new OI();
    }
    return INSTANCE;
  }

  private final Joystick driver = new Joystick(ControllerMap.DRIVER_PORT);
  private final Joystick operator = new Joystick(ControllerMap.OPERATOR_PORT);

  private OI() {
    //Driver Controls

      new JoystickButton(driver, ControllerMap.RB).whenPressed(new driveByCamera(CAMERA.FRONT));
      new JoystickButton(driver, ControllerMap.RB).whenPressed(new driveByCamera(CAMERA.BACK));
      // new JoystickButton(driver, ControllerMap.Y).whileHeld(new driveByVision(CAMERA.FRONT));
      // new JoystickButton(driver, ControllerMap.A).whileHeld(new driveByVision(CAMERA.BACK));
      new JoystickButton(driver, ControllerMap.Y).whileHeld(new driveByDockingPID(CAMERA.FRONT));
      new JoystickButton(driver, ControllerMap.A).whileHeld(new driveByDockingPID(CAMERA.BACK));
      new JoystickButton(driver, ControllerMap.LB).whileHeld(new FollowArcTesting());
      new JoystickButton(driver, ControllerMap.B).whenPressed(new ReleaseCargo());
      new JoystickButton(driver, ControllerMap.X).whenPressed(new ReleaseHatch());
      new JoystickButton(driver, ControllerMap.LOGO_LEFT).whenPressed(new stopCargoGrabber());
      new JoystickButton(driver, ControllerMap.LOGO_RIGHT).whenPressed(new unDockArm());



    //Operator Buttons

      new JoystickButton(operator, ControllerMap.LOGO_LEFT).whileHeld(new burstExtendLifter());
      new JoystickButton(operator, ControllerMap.LOGO_RIGHT).whenPressed(new ExtendLifter());
      // new JoystickButton(operator, ControllerMap.A).whileHeld(new reverseLifter());
      new JoystickButton(operator, ControllerMap.RB).whenPressed(new PickUpCargo());
      new JoystickButton(operator, ControllerMap.RB).whenReleased(new resetCargoJester());
      new JoystickButton(operator, ControllerMap.LB).whenPressed(new ReleaseCargo());
      new JoystickButton(operator, ControllerMap.X).whenPressed(new GrabCargo());
      new JoystickButton(operator, ControllerMap.B).whenPressed(new ReleaseCargo());
      new JoystickButton(operator, ControllerMap.LEFT_STICK_BUTTON).whenPressed(new stopCargoGrabber());
      new JoystickButton(operator, ControllerMap.Y).whenPressed(new ReleaseHatch());
      new JoystickButton(operator, ControllerMap.A).whenPressed(new GrabHatch());


      // new JoystickButton(operator, ControllerMap.Y).whenPressed(new openGrabber());


      new Button() {

        @Override
        public boolean get() {
          return (operator.getPOV() == 0);
        }
      }.whenPressed(new upCommand());

      new Button() {

        @Override
        public boolean get() {
          return operator.getPOV() == 180;
        }
      }.whenPressed(new downCommand());

      new Button() {

        @Override
        public boolean get() {
          return operator.getPOV() == 90;
        }
      }.whenPressed(new ToggleArmCommand());
  }

  /**
   * @return the raw controller throttle
   */
  public double getThrottle() {
    return -driver.getRawAxis(ControllerMap.LEFT_STICK_Y); 
    // return -driver.getRawAxis(ControllerMap.LEFT_TRIGGER) + driver.getRawAxis(ControllerMap.RIGHT_TRIGGER);
	}
  
  /**
   * @return the raw controller turn
   */
  public double getTurn() {
    return driver.getRawAxis(ControllerMap.RIGHT_STICK_X);
  }
  
  public double getArmPower() {
    double stick = -operator.getRawAxis(ControllerMap.LEFT_STICK_Y);
    stick *= Math.abs(stick);
    if (Math.abs(stick) < 0.05) {
      stick = 0;
    }
    return stick;
  }

  public double getGMPOV() {
    return operator.getPOV();
  }
  
  /**
	 * Turns on and off the rumble function on the driver and operator controllers
	 * @param set true to turn on rumble
	 */
	public void setControllerRumble(boolean state) {
		if (state == true) {
			driver.setRumble(RumbleType.kLeftRumble, 1);
			driver.setRumble(RumbleType.kRightRumble, 1);  
			operator.setRumble(RumbleType.kLeftRumble, 1);
			operator.setRumble(RumbleType.kRightRumble, 1);
		} else {
			driver.setRumble(RumbleType.kLeftRumble, 0);
			driver.setRumble(RumbleType.kRightRumble, 0);
			operator.setRumble(RumbleType.kLeftRumble, 0);
			operator.setRumble(RumbleType.kRightRumble, 0);
		}
	}
}