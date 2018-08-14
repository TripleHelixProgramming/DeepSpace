package frc.robot.commands.drivetrain;

import frc.robot.RobotMap;

public class JoystickDriveFactory {
    public static JoystickDrive createJoystickDrive() {
        switch (RobotMap.currentDriver) {
            case DUSTIN:
            default:
                return new JoystickDrive();
        }
    }
}