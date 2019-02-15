// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.jester_arm;

// import com.team2363.logger.HelixEvents;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.subsystems.JesterArm;
// import frc.robot.subsystems.JesterArm.ArmPos;

// public class ToggleArmCommand extends Command {
//   public ToggleArmCommand() {
//     // Use requires() here to declare subsystem dependencies
//     // eg. requires(chassis);
//     requires(JesterArm.getInstance());
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {
//     HelixEvents.getInstance().addEvent("TOGGLE_ARM", "Starting to toggle arm");
//   }

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   protected void execute() {
    
//     ArmPos currentArmPos = JesterArm.getInstance().getCurrentArmPreset();

//     switch (currentArmPos) {
//     case FRONT_HATCH_LOWER:
//       JesterArm.getInstance().setArmMotionMagic(ArmPos.BACK_HATCH_LOWER);
//       break;
//     case BACK_HATCH_LOWER:
//       JesterArm.getInstance().setArmMotionMagic(ArmPos.FRONT_HATCH_LOWER);
//       break;
//     case FRONT_HATCH_MIDDLE:
//       JesterArm.getInstance().setArmMotionMagic(ArmPos.BACK_HATCH_MIDDLE);
//       break;
//     case BACK_HATCH_MIDDLE:
//       JesterArm.getInstance().setArmMotionMagic(ArmPos.FRONT_HATCH_MIDDLE);
//       break;
//     case FRONT_BALL_LOWER:
//       JesterArm.getInstance().setArmMotionMagic(ArmPos.BACK_BALL_LOWER);
//       break;
//     case BACK_BALL_LOWER:
//       JesterArm.getInstance().setArmMotionMagic(ArmPos.FRONT_BALL_LOWER);
//       break;
//     case FRONT_BALL_MIDDLE:
//       JesterArm.getInstance().setArmMotionMagic(ArmPos.BACK_BALL_MIDDLE);
//       break;
//     case BACK_BALL_MIDDLE:
//       JesterArm.getInstance().setArmMotionMagic(ArmPos.FRONT_BALL_MIDDLE);
//       break;
//     default:
//       JesterArm.getInstance().setArmMotionMagic(ArmPos.FRONT_HATCH_LOWER);
//       break;
//     }
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   protected boolean isFinished() {
//     return true;
//   }

//   // Called once after isFinished returns true
//   @Override
//   protected void end() {
//     HelixEvents.getInstance().addEvent("TOGGLE_ARM", "Ending toggle arm");
//   }

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   protected void interrupted() {
//   }
// }
