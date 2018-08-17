package frc.robot.commands.drivetrain;

public class DustinDrive extends JoystickDrive {
    
    @Override
    protected double getThrottleScalar() {
        return 0.8;
    }

    @Override
    protected double getTurnScalar() {
        return 0.6;
    }
}