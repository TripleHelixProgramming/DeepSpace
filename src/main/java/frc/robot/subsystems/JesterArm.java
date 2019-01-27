/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team2363.logger.HelixLogger;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.jester_arm.DriveArmByJoystick;

public class JesterArm extends Subsystem {

    private TalonSRX armMotor;
    public static int ARM_ACCELERATION = 100;
    public static int ARM_CRUISE = 300;



    public enum ArmPos {
        START(200),
        FRONT_LOWER(ArmPos.START.pos+100),
        FRONT_MIDDLE(ArmPos.START.pos+200),
        FRONT_UPPER(ArmPos.START.pos+300),
        BACK_UPPER(ArmPos.START.pos+400),
        BACK_MIDDLE(ArmPos.START.pos+500),
        BACK_LOWER(ArmPos.START.pos+600),
        BACK_LIMIT(ArmPos.START.pos+650);

        public final int pos;

        ArmPos(int pos) {
            this.pos = pos;
        }
    }

    private static JesterArm INSTANCE = new JesterArm();
    public ArmPos currentArmPreset = ArmPos.START;

    /**
     * @return the singleton instance of the JesterArm subsystem
     */
    public static JesterArm getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new JesterArm();
        }
        return INSTANCE;
    }

    public JesterArm() {
        super("Jester Arm Subsystem");
        setupLogs();

        currentArmPreset = ArmPos.START;

        armMotor = new TalonSRX(RobotMap.SHOULDER_MASTER_TALON_ID);

        armMotor.configContinuousCurrentLimit(40, 0);
        armMotor.configPeakCurrentLimit(60, 0);
        armMotor.configPeakCurrentDuration(100, 0);
        armMotor.enableCurrentLimit(true);

        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, RobotMap.CTRE_TIMEOUT_INIT);
        armMotor.config_kP(0, 1, RobotMap.CTRE_TIMEOUT_INIT);
        armMotor.config_kI(0, 0.01, RobotMap.CTRE_TIMEOUT_INIT);

        armMotor.setNeutralMode(NeutralMode.Coast);

        setArmSoftLimits(ArmPos.START.pos, ArmPos.BACK_LIMIT.pos);
        setArmMotionProfile(ARM_ACCELERATION, ARM_CRUISE);

        // The arm starts the match in a one-time docked position.  Move arm from
        // docked position to front lower scoring position.
        unDockArm();
    }

    private void setupLogs() {
        HelixLogger.getInstance().addDoubleSource("ARM VOLTAGE", armMotor::getMotorOutputVoltage);
    }

    public void setArmHeight(int pos) {
        armMotor.set(ControlMode.Position, pos);
    }

    
    // Move arm from docked position (at start of match) to front lower scoring position.
    public void unDockArm() {
        setArmMotionMagic(ArmPos.FRONT_LOWER.pos);
        currentArmPreset = ArmPos.FRONT_LOWER;
    }

    // Swing arm to opposite side keeping current arm height.
    public void toggleArm() {

        switch (currentArmPreset) {
            case FRONT_LOWER:
                setArmMotionMagic(ArmPos.BACK_LOWER.pos);
                currentArmPreset = ArmPos.BACK_LOWER;
                break;
            case BACK_LOWER: 
                setArmMotionMagic(ArmPos.FRONT_LOWER.pos);
                currentArmPreset = ArmPos.FRONT_LOWER;
                break;
            case FRONT_MIDDLE: 
                setArmMotionMagic(ArmPos.BACK_MIDDLE.pos);
                currentArmPreset = ArmPos.BACK_MIDDLE;
                break;
            case BACK_MIDDLE: 
                setArmMotionMagic(ArmPos.FRONT_MIDDLE.pos);
                currentArmPreset = ArmPos.FRONT_MIDDLE;
                break;
            default:
                setArmMotionMagic(ArmPos.FRONT_LOWER.pos);
                currentArmPreset = ArmPos.FRONT_LOWER;
                break;
        }
    }

    // Swap to opposite height on current arm side. Later will us up & down
    public void toggleHeight() {

        switch (currentArmPreset) {
            case FRONT_LOWER:
                setArmMotionMagic(ArmPos.FRONT_MIDDLE.pos);
                currentArmPreset = ArmPos.FRONT_MIDDLE;
                break;
            case FRONT_MIDDLE:
                setArmMotionMagic(ArmPos.FRONT_LOWER.pos);
                currentArmPreset = ArmPos.FRONT_LOWER;
                break;               
            case BACK_LOWER:
                setArmMotionMagic(ArmPos.BACK_MIDDLE.pos);
                currentArmPreset = ArmPos.BACK_MIDDLE;
                break;
            case BACK_MIDDLE:
                setArmMotionMagic(ArmPos.BACK_LOWER.pos);
                currentArmPreset = ArmPos.BACK_LOWER;
                break;
            default:
                setArmMotionMagic(ArmPos.FRONT_LOWER.pos);
                currentArmPreset = ArmPos.FRONT_LOWER;
                break;
        }
    }

    public void setArmMotionMagic(int pos) {
        // armMotor.set(ControlMode.MotionMagic, pos);
    }

    public void driveArmPercentOut(double percent) {
        armMotor.set(ControlMode.PercentOutput, percent);
    }

    public void goTo(ArmPos armPos) {
        setArmMotionMagic(armPos.pos);
    }

    public int getArmPos() {
        return armMotor.getSelectedSensorPosition(0);
    }

    public void setArmSoftLimits(int reverseSoftLimit, int forwardSoftLimit) {
        armMotor.configReverseSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        armMotor.configReverseSoftLimitThreshold(reverseSoftLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);

        armMotor.configForwardSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        armMotor.configForwardSoftLimitThreshold(forwardSoftLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);
    }

    public void setArmMotionProfile(int acceleration, int cruise) {
        armMotor.configMotionAcceleration(acceleration, RobotMap.CTRE_TIMEOUT_INIT);
        armMotor.configMotionCruiseVelocity(cruise, RobotMap.CTRE_TIMEOUT_INIT);
    }

    // Put items in here that you want updated on SmartDash during disableperiodic()
    public void updateSmartDash() {
        SmartDashboard.putNumber("Arm Pos", getArmPos());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Pos", getArmPos());
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveArmByJoystick());
    }
}