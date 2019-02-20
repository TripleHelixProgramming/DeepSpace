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
import frc.robot.commands.jester_wrist.FollowArm;
import frc.robot.commands.jester_wrist.StopWrist;
import frc.robot.subsystems.JesterArm;
import frc.robot.ArmPreset;

public class JesterWrist extends Subsystem {

    private TalonSRX wristMotor = new TalonSRX(RobotMap.WRIST_ID);

    public static int WRIST_ACCELERATION = 180;
    public static int WRIST_CRUISE = 25;

    private static JesterWrist INSTANCE = new JesterWrist();

    /**
     * @return the singleton instance of the JesterArm subsystem
     */
    public static JesterWrist getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new JesterWrist();
        }
        return INSTANCE;
    }

    public JesterWrist() {
        super("Jester Wrist Subsystem");

        // setupLogs();
        wristMotor.configFactoryDefault();

        // Protect the motors and protect from brown out
        wristMotor.configContinuousCurrentLimit(40, 0);
        wristMotor.configPeakCurrentLimit(60, 0);
        wristMotor.configPeakCurrentDuration(100, 0);
        wristMotor.enableCurrentLimit(true);

        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, RobotMap.CTRE_TIMEOUT_INIT);
        wristMotor.configFeedbackNotContinuous(true, RobotMap.CTRE_TIMEOUT_INIT);

        // Need to verify and set. With positive motor direction sensor values should
        // increase.
        wristMotor.setSensorPhase(false);
        wristMotor.setInverted(false);

        // PID settings
        wristMotor.config_kF(0, 0.0, RobotMap.CTRE_TIMEOUT_INIT);
        wristMotor.config_kP(0, 96.0, RobotMap.CTRE_TIMEOUT_INIT);
        wristMotor.config_kI(0, 0.0, RobotMap.CTRE_TIMEOUT_INIT);
        wristMotor.config_kD(0, 0.0, RobotMap.CTRE_TIMEOUT_INIT);
        wristMotor.configAllowableClosedloopError(0, 0, RobotMap.CTRE_TIMEOUT_INIT);

        setWristSoftLimits();
        setWristMotionProfile(WRIST_ACCELERATION, WRIST_CRUISE);
    }

    public void setWristSoftLimits() {

        int lowerLimit, upperLimit;
        ArmPreset curArmPreset; 

        // Dynamially set wrist soft limits as arm positions changes.
        curArmPreset = JesterArm.getInstance().getCurrentArmPreset();
        lowerLimit = curArmPreset.getWristLowerLimit(curArmPreset.getShoulderAngle());
        upperLimit = curArmPreset.getWristUpperLimit(curArmPreset.getShoulderAngle());

        wristMotor.configReverseSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        wristMotor.configReverseSoftLimitThreshold(lowerLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);

        wristMotor.configForwardSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        wristMotor.configForwardSoftLimitThreshold(upperLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);

        wristMotor.configClosedloopRamp(0.2,0);

        SmartDashboard.putNumber("Wrist Lower Limit", lowerLimit);
        SmartDashboard.putNumber("Wrist Upper Limit", upperLimit);
    }

    public int getWristPos() {
        return wristMotor.getSelectedSensorPosition(0);
    }

    public int getWristError() {
        return wristMotor.getClosedLoopError(0);
    }

    private void setWristMotionProfile(int acceleration, int cruise) {
        wristMotor.configMotionAcceleration(acceleration, RobotMap.CTRE_TIMEOUT_INIT);
        wristMotor.configMotionCruiseVelocity(cruise, RobotMap.CTRE_TIMEOUT_INIT);
    }

    public void setWristPos(ArmPreset preset) {
        int newPos = preset.CalculateWristPos();

        SmartDashboard.putNumber("Calc Wrist Pos", newPos);
        setWristMotionMagic(newPos);
    }

    public void GoToRelatedPreset(int armPos) {

        for (ArmPreset preset : ArmPreset.values()) {
            if (preset.getShoulderAngle() == preset.getShoulderAngle(armPos)) {
                SmartDashboard.putString("Related Preset", preset.toString());
                setWristPos(preset);
                return;
            }
        }
    }

    public void setWristMotionMagic(int pos) {
        wristMotor.set(ControlMode.MotionMagic, pos);
    }

    public void stop() {
        wristMotor.set(ControlMode.PercentOutput, 0.0);
    }

    // Put items in here that you want updated on SmartDash during disableperiodic()
    public void updateSmartDash() {
        SmartDashboard.putNumber("Wrist Pos", getWristPos());
    }

    @Override
    public void periodic() {
        setWristSoftLimits();
        updateSmartDash();
    }

    private void setupLogs() {
        HelixLogger.getInstance().addDoubleSource("WRIST VOLTAGE", wristMotor::getMotorOutputVoltage);
    }

    public void driveWristPercentOut(double percent) {
        wristMotor.set(ControlMode.PercentOutput, percent);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new FollowArm());
        // setDefaultCommand(new StopWrist());
    }
}
