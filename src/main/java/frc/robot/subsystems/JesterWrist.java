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
import frc.robot.subsystems.JesterArm.ArmPos;

public class JesterWrist extends Subsystem {

    private TalonSRX wristMotor;
    public static int WRIST_ACCELERATION = 100;
    public static int WRIST_CRUISE = 300;

    // Standard wrist positions.  This assumes the pot is mounted such that lower values 
    // correspond to front arm positions.
    public enum Wrist {
        START(100),                             // Wrist starting position
        FRONT_LIMIT(Wrist.START.pos + 100),      // True Limit given to PID control
        FRONT(Wrist.START.pos + 300),           // Normal Position when arm is to the front
        TRANSITION(Wrist.START.pos + 400),      // Position to in when above the highest hatch level
        BACK(Wrist.START.pos + 500),            // Normal Position when arm is to back
        BACK_LIMIT(Wrist.START.pos + 700);      // Ture Limit given to PID Control

        public final int pos;

        Wrist(int pos) {
            this.pos = pos;
        }
    }

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

        setupLogs();

        wristMotor = new TalonSRX(RobotMap.WRIST_TALON_ID);

        // Protect the motors and protect from brown out
        wristMotor.configContinuousCurrentLimit(40, 0);
        wristMotor.configPeakCurrentLimit(60, 0);
        wristMotor.configPeakCurrentDuration(100, 0);
        wristMotor.enableCurrentLimit(true);

        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, RobotMap.CTRE_TIMEOUT_INIT);

        wristMotor.config_kP(0, 1, RobotMap.CTRE_TIMEOUT_INIT);
        wristMotor.config_kI(0, 0.01, RobotMap.CTRE_TIMEOUT_INIT);

        setWristSoftLimits(Wrist.FRONT_LIMIT.pos, Wrist.BACK_LIMIT.pos);
        setWristMotionProfile(WRIST_ACCELERATION, WRIST_CRUISE);

        wristMotor.configAllowableClosedloopError(0, 2, RobotMap.CTRE_TIMEOUT_INIT);
    }

    public int getWristVelocity() {
        return wristMotor.getSelectedSensorVelocity();
      }

    private void setupLogs() {
        HelixLogger.getInstance().addDoubleSource("WRIST VOLTAGE", wristMotor::getMotorOutputVoltage);
        HelixLogger.getInstance().addIntegerSource("WRIST VELOCITY", JesterWrist.getInstance()::getWristVelocity);
    }

    public void driveWristPercentOut(double percent) {
        wristMotor.set(ControlMode.PercentOutput, percent);
    }

    public void setWristSoftLimits(int reverseSoftLimit, int forwardSoftLimit) {
        wristMotor.configReverseSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        wristMotor.configReverseSoftLimitThreshold(reverseSoftLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);

        wristMotor.configForwardSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        wristMotor.configForwardSoftLimitThreshold(forwardSoftLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Wrist Pos", getWristPos());
    }

    public int getWristPos() {
        return wristMotor.getSelectedSensorPosition(0);
    }

    public int getWristError() {
        return wristMotor.getClosedLoopError(0);
    }

    // Calculate the wrist position based on the arm position.
    public void setWristPos(int pos) {
        setWristMotionMagic(pos);
    }

    private void setWristMotionProfile(int acceleration, int cruise) {
        wristMotor.configMotionAcceleration(acceleration, RobotMap.CTRE_TIMEOUT_INIT);
        wristMotor.configMotionCruiseVelocity(cruise, RobotMap.CTRE_TIMEOUT_INIT);
    }

    private void setWristMotionMagic(int pos) {
        SmartDashboard.putNumber("Wrist Pos", pos);
        // wristMotor.set(ControlMode.MotionMagic, pos);
    }

    // Put items in here that you want updated on SmartDash during disableperiodic()
    public void updateSmartDash() {
        SmartDashboard.putNumber("Wrist Pos", getWristPos());
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new FollowArm());
    }
    
}
