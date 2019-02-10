package frc.robot.commands.jester_arm;

import com.team2363.logger.HelixEvents;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.subsystems.JesterArm;
import frc.robot.subsystems.JesterWrist;
import frc.robot.subsystems.JesterArm.ArmPos;

public class DriveArmByJoystick extends Command {

    private JesterArm jesterArm = JesterArm.getInstance();
    private JesterWrist jesterWrist = JesterWrist.getInstance();

    int position;

    public DriveArmByJoystick() {
        super("Drive arm by joystick");
        requires(jesterArm);
        requires(jesterWrist);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        position = jesterArm.getArmPos();
        HelixEvents.getInstance().addEvent("JESTER ARM", "Starting to drive arm by joystick");
    }

    @Override
    protected void execute() {

        double error;

        position += OI.getInstance().getArmPower() * 20;
        if (position > ArmPos.BACK_LIMIT.pos) {
            position = ArmPos.BACK_LIMIT.pos;
        } else if (position < 0) {
            position = 0;
        }

        jesterArm.setArmMotionMagic(position);

        error = position - jesterArm.getArmPos();
        SmartDashboard.putNumber("Arm Manual Position", position);
        SmartDashboard.putNumber("Arm Error", error);

        // some quick code to test the wrist
        double pov = OI.getInstance().getGMPOV();
        if (pov == 0) {
            jesterWrist.driveWristPercentOut(0.4);
        } else if (pov == 180) {
            jesterWrist.driveWristPercentOut(-0.4);
        } else {
            jesterWrist.driveWristPercentOut(0);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        HelixEvents.getInstance().addEvent("JESTER ARM", "finished DriveArmByJoystick");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        HelixEvents.getInstance().addEvent("JESTER ARM", "Interrupted DriveArmByJoystick");
    }
}
