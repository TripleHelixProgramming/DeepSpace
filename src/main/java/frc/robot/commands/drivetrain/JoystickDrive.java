package frc.robot.commands.drivetrain;

import static frc.robot.subsystems.Drivetrain.DT_HALF_TRACK_WIDTH;
import static frc.robot.subsystems.Drivetrain.MAX_DRIVESIDE_VELOCITY;
import static frc.robot.subsystems.Drivetrain.ticks_per_100ms;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

/**
 *
 */
public class JoystickDrive extends Command {

    public JoystickDrive() {
        // Use requires() here to declare subsystem dependencies
		requires(Drivetrain.getInstance());
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	HelixEvents.getInstance().addEvent("DRIVETRAIN", "Starting: " + JoystickDrive.class.getSimpleName());
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    	//read in joystick values from OI
    	//range [-1, 1]
    	double throttleInput = OI.getInstance().getThrottle() * getThrottleScalar();
		double turnInput = OI.getInstance().getTurn() * getTurnScalar();
		
		// double throttleInput = OI.getInstance().getThrottle();
    	// double turnInput = OI.getInstance().getTurn();
 
    	//find the maximum possible value of (throttle + turn)
    	//along the vector that the arcade joystick is pointing
    	double saturatedInput;
    	double greaterInput = Math.max(Math.abs(throttleInput), Math.abs(turnInput));
    		//range [0, 1]
    	double lesserInput = Math.abs(throttleInput) + Math.abs(turnInput) - greaterInput;
    		//range [0, 1]
    	if (greaterInput > 0.0) {
    		saturatedInput = (lesserInput / greaterInput) + 1.0;
       		//range [1, 2]
    	} else {
    		saturatedInput = 1.0;
    	}
     	
    	//scale down the joystick input values
		//such that (throttle + turn) always has a range [-1, 1]
    	throttleInput = throttleInput / saturatedInput;
		turnInput = turnInput / saturatedInput;
		// throttleInput = throttleInput / saturatedInput * getThrottleScalar();
    	// turnInput = turnInput / saturatedInput * getTurnScalar();
     	
    	double radialVelocityAtMidpoint = throttleInput * MAX_DRIVESIDE_VELOCITY;
    		//range [-full linear speed, full linear speed]
    		//units of linear speed (in/s)
    	double angularVelocity =  turnInput * MAX_DRIVESIDE_VELOCITY / DT_HALF_TRACK_WIDTH; 
    		//range [-full rotational speed, full rotational speed]
    		//units of rotational speed (rad/s);
    	double radialVelocityAtDriveside = angularVelocity * DT_HALF_TRACK_WIDTH;
    		//range [-full linear speed, full linear speed]
			//units of linear speed (in/s); 
      	
    	//double left = (radialVelocityAtMidpoint + radialVelocityAtDriveside) * ticks_per_100ms;
    	//double right = (radialVelocityAtMidpoint - radialVelocityAtDriveside) * ticks_per_100ms;
		
		double left = throttleInput + turnInput;
		double right = throttleInput - turnInput;
	    Drivetrain.getInstance().tankDrive(left, right);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	HelixEvents.getInstance().addEvent("DRIVETRAIN", "Finished starting joystick drive");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
	}
	
	protected double getThrottleScalar() {
		return 1;
	}

	protected double getTurnScalar() {
		return 1;
	}
}