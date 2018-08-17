package frc.robot.commands.drivetrain;

import frc.logger.HelixEvents;
import frc.robot.RobotMap;

public class JoystickDriveFactory {
    
    public static JoystickDrive createJoystickDrive() {
        switch (RobotMap.currentDriver) {
            case DUSTIN:
                HelixEvents.getInstance().addEvent("DRIVETRAIN", "Creating a Dustin Drive");
                return new DustinDrive();
            default:
            HelixEvents.getInstance().addEvent("DRIVETRAIN", "Creating a Default Drive");
                return new JoystickDrive();
        }
    }
}