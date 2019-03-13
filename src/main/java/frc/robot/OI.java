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
import frc.robot.commands.Auto.DeliverMiddleGroup;
import frc.robot.commands.Auto.FollowArcTesting;
import frc.robot.commands.Auto.PickUpCargo;
import frc.robot.commands.Auto.resetCargoJester;
import frc.robot.commands.Auto.undockJester;
import frc.robot.commands.cargo_grabber.GrabCargo;
import frc.robot.commands.cargo_grabber.ReleaseCargo;
import frc.robot.commands.cargo_grabber.stopCargoGrabber;
import frc.robot.commands.drivetrain.driveByAssist;
import frc.robot.commands.drivetrain.driveByAssistJosh;
import frc.robot.commands.drivetrain.driveByCamera;
import frc.robot.commands.drivetrain.driveByDocking;
import frc.robot.commands.drivetrain.driveByDockingPID;
import frc.robot.commands.hatch.GrabHatch;
import frc.robot.commands.hatch.ReleaseHatch;
import frc.robot.commands.jester_arm.MoveToLower;
import frc.robot.commands.jester_arm.MoveToMiddle;
import frc.robot.commands.jester_arm.MoveToPickup;
import frc.robot.commands.jester_arm.MoveToUpper;
import frc.robot.commands.robot_lifter.ExtendLifter;
import frc.robot.commands.robot_lifter.burstExtendLifter;
import frc.robot.commands.robot_lifter.reverseLifter;
import frc.robot.commands.robot_lifter.StopLifter;

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
      new JoystickButton(driver, ControllerMap.Y).whileHeld(new driveByAssistJosh(CAMERA.FRONT));
      new JoystickButton(driver, ControllerMap.A).whileHeld(new driveByAssistJosh(CAMERA.BACK));
      //new JoystickButton(driver, ControllerMap.LB).whenPressed(new undockJester());
      new JoystickButton(driver, ControllerMap.B).whenPressed(new ReleaseCargo());
      new JoystickButton(driver, ControllerMap.X).whenPressed(new ReleaseHatch());
      new JoystickButton(driver, ControllerMap.LOGO_LEFT).whenPressed(new stopCargoGrabber());
      new JoystickButton(driver, ControllerMap.LOGO_RIGHT).whenPressed(new undockJester());



    //Operator Buttons

      // new JoystickButton(operator, ControllerMap.LOGO_LEFT).whenPressed(new burstExtendLifter());
      // new JoystickButton(operator, ControllerMap.LOGO_LEFT).whenReleased(new StopLifter());
      // new JoystickButton(operator, ControllerMap.LOGO_RIGHT).whenPressed(new ExtendLifter());
      // new JoystickButton(operator, ControllerMap.LOGO_RIGHT).whenReleased(new StopLifter());
      // new JoystickButton(operator, ControllerMap.A).whileHeld(new reverseLifter());
      // new JoystickButton(operator, ControllerMap.RB).whenPressed(new PickUpCargo());
      // new JoystickButton(operator, ControllerMap.RB).whenReleased(new resetCargoJester());
      // new JoystickButton(operator, ControllerMap.X).whenPressed(new GrabHatch());
      // new JoystickButton(operator, ControllerMap.B).whenPressed(new ReleaseCargo());
      // new JoystickButton(operator, ControllerMap.LEFT_STICK_BUTTON).whenPressed(new stopCargoGrabber());
      // new JoystickButton(operator, ControllerMap.Y).whenPressed(new ReleaseHatch());
      // new JoystickButton(operator, ControllerMap.A).whenPressed(new GrabCargo());

      new JoystickButton(operator, ControllerMap.PS4_SHARE).whileHeld(new burstExtendLifter());
      new JoystickButton(operator, ControllerMap.PS4_OPTIONS).whenPressed(new ExtendLifter());
      // new JoystickButton(operator, ControllerMap.A).whileHeld(new reverseLifter());
      new JoystickButton(operator, ControllerMap.PS4_R1).whenPressed(new PickUpCargo());
      new JoystickButton(operator, ControllerMap.PS4_R1).whenReleased(new resetCargoJester());
      new JoystickButton(operator, ControllerMap.PS4_SQUARE).whenPressed(new GrabHatch());
      new JoystickButton(operator, ControllerMap.PS4_CIRCLE).whenPressed(new ReleaseCargo());
      new JoystickButton(operator, ControllerMap.PS4_L3).whenPressed(new stopCargoGrabber());
      new JoystickButton(operator, ControllerMap.PS4_TRIANGLE).whenPressed(new ReleaseHatch());
      new JoystickButton(operator, ControllerMap.PS4_X).whenPressed(new GrabCargo());


      // new JoystickButton(operator, ControllerMap.Y).whenPressed(new openGrabber());


      new Button() {

        @Override
        public boolean get() {
          return (operator.getPOV() == 0);
        }
      }.whenPressed(new MoveToUpper());

      new Button() {

        @Override
        public boolean get() {
          return operator.getPOV() == 180;
        }
      }.whenPressed(new MoveToLower());

      new Button() {

        @Override
        public boolean get() {
          return operator.getPOV() == 90;
        }
      }.whenPressed(new MoveToMiddle());

      new Button() {

        @Override
        public boolean get() {
          return operator.getPOV() == 270;
        }
      }.whenPressed(new MoveToPickup());
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

  // public double getGMPOV() {
  //   return operator.getPOV();
  // }
  
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