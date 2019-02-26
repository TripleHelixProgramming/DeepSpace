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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team2363.logger.HelixLogger;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.ArmPreset;

public class JesterArm extends Subsystem {

    private TalonSRX armMaster = new TalonSRX(RobotMap.ARM_MASTER_ID);
    private VictorSPX armSlave = new VictorSPX(RobotMap.ARM_SLAVE_ID);

    public static int ARM_ACCELERATION = 30;
    public static int ARM_CRUISE = 3;

    private static JesterArm INSTANCE = new JesterArm();

    private ArmPreset currentArmPreset = ArmPreset.START;

    private ArmPreset lastUpperPos = ArmPreset.DELIVER_BALL_UPPER;
    private ArmPreset lastMiddlePos = ArmPreset.DELIVER_BALL_MIDDLE;
    private ArmPreset lastLowerPos = ArmPreset.DELIVER_BALL_LOWER;
    private ArmPreset lastPickupPos = ArmPreset.CARGO;

    // States which allow only certain destinations based on what the robot is
    // carrrying
    public enum BotState {
        EMPTY, BALL, HATCH
    };

    // Bot is holding a hatch at the start of match
    private BotState curBotState = BotState.HATCH; 

    private int reverseSoftLimit, fwdSoftLimit;

    PowerDistributionPanel pdp = new PowerDistributionPanel();

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
        // setupLogs();

        armSlave.configFactoryDefault();
        armMaster.configFactoryDefault();

        // Setting initial state modes -  Holding Hatch at START of match
        currentArmPreset = ArmPreset.START;
        curBotState = BotState.HATCH;
        // Since holding HATCH at beginning, last settings for dest settings should be BALL states.
        lastUpperPos = ArmPreset.DELIVER_BALL_UPPER;
        lastMiddlePos = ArmPreset.DELIVER_BALL_MIDDLE;
        lastLowerPos = ArmPreset.DELIVER_BALL_LOWER;
        lastPickupPos = ArmPreset.CARGO;

        armSlave.follow(armMaster);
        armSlave.setNeutralMode(NeutralMode.Brake);
        armSlave.configOpenloopRamp(0.2, 0);

        armMaster.setNeutralMode(NeutralMode.Brake);
        armMaster.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.configFeedbackNotContinuous(true, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.configClosedloopRamp(0.2, 0);

        // Need to verify and set. With positive motor direction sensor values should
        // increase.
        armMaster.setSensorPhase(true);
        armMaster.setInverted(false);

        // PID Settings for Competition Bot
        armMaster.config_kF(0, 0.0, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.config_kP(0, 23.0, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.config_kI(0, 0.0, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.config_kD(0, 10.0, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.configAllowableClosedloopError(0, 0, RobotMap.CTRE_TIMEOUT_INIT);

        // Set current limiting
        armMaster.configContinuousCurrentLimit(40, 0);
        armMaster.configPeakCurrentLimit(60, 0);
        armMaster.configPeakCurrentDuration(100, 0);
        armMaster.enableCurrentLimit(true);

        reverseSoftLimit = ArmPreset.PICKUP_HATCH.CalculateArmPos(); // lower sensor reading side
        fwdSoftLimit = ArmPreset.DELIVER_HATCH_LOWER.CalculateArmPos(); // higher sensor reading side
        setArmSoftLimits(reverseSoftLimit, fwdSoftLimit);
        setArmMotionProfile(ARM_ACCELERATION, ARM_CRUISE);
    }

    private void setupLogs() {
        HelixLogger.getInstance().addDoubleSource("ARM MASTER CURRENT", armMaster::getOutputCurrent);
        // HelixLogger.getInstance().addDoubleSource("ARM SLAVE", () ->
        // pdp.getCurrent(RobotMap.ARM_SLAVE_ID));
    }

    public void goTo(ArmPreset preset) {
        int newPos = preset.CalculateArmPos();
        currentArmPreset = preset;
        SmartDashboard.putString("goTo: ", preset.toString());
        SmartDashboard.putNumber("Calc Arm Pos", newPos);
        setArmMotionMagic(newPos);
    }

    public void resetArmPreset() {
        currentArmPreset = ArmPreset.START;
    }

    public void setState(BotState bot_state) {
        curBotState = bot_state;
    }

    public BotState getState() {
        return(curBotState);
    }

    public boolean isLastMoveDone() {
        return(Math.abs(getArmPos() - currentArmPreset.CalculateArmPos()) <= 2);
    }

    public void upperPos() {
        switch (curBotState) {
        case BALL:
            goTo(ArmPreset.DELIVER_BALL_UPPER);
            lastUpperPos = ArmPreset.DELIVER_BALL_UPPER;
            break;
        case HATCH:
            goTo(ArmPreset.DELIVER_HATCH_UPPER);
            lastUpperPos = ArmPreset.DELIVER_HATCH_UPPER;
            break;
        case EMPTY:
        default:
            if (lastUpperPos == ArmPreset.DELIVER_BALL_UPPER) {
                goTo(ArmPreset.DELIVER_HATCH_UPPER);
                lastUpperPos = ArmPreset.DELIVER_HATCH_UPPER;
            } else {
                goTo(ArmPreset.DELIVER_BALL_UPPER);
                lastUpperPos = ArmPreset.DELIVER_BALL_UPPER;
            }
            break;
        }
    }

    public void middlePos() {
        switch (curBotState) {
        case BALL:
            goTo(ArmPreset.DELIVER_BALL_MIDDLE);
            lastMiddlePos = ArmPreset.DELIVER_BALL_MIDDLE;
            break;
        case HATCH:
            goTo(ArmPreset.DELIVER_HATCH_MIDDLE);
            lastMiddlePos = ArmPreset.DELIVER_HATCH_MIDDLE;
            break;
        case EMPTY:
        default:
            if (lastMiddlePos == ArmPreset.DELIVER_BALL_MIDDLE) {
                goTo(ArmPreset.DELIVER_HATCH_MIDDLE);
                lastMiddlePos = ArmPreset.DELIVER_HATCH_MIDDLE;
            } else {
                goTo(ArmPreset.DELIVER_BALL_MIDDLE);
                lastMiddlePos = ArmPreset.DELIVER_BALL_MIDDLE;
            }
            break;
        }
    }

    public void lowerPos() {
        switch (curBotState) {
        case BALL:
            goTo(ArmPreset.DELIVER_BALL_LOWER);
            lastLowerPos = ArmPreset.DELIVER_BALL_LOWER;
            break;
        case HATCH:
            goTo(ArmPreset.DELIVER_HATCH_LOWER);
            lastLowerPos = ArmPreset.DELIVER_HATCH_LOWER;
            break;
        case EMPTY:
        default:
            if (lastLowerPos == ArmPreset.DELIVER_BALL_LOWER) {
                goTo(ArmPreset.DELIVER_HATCH_LOWER);
                lastLowerPos = ArmPreset.DELIVER_HATCH_LOWER;
            } else {
                goTo(ArmPreset.DELIVER_BALL_LOWER);
                lastLowerPos = ArmPreset.DELIVER_BALL_LOWER;
            }
            break;
        }
    }

    public void pickupPos() {
        switch (curBotState) {
        case BALL:
            goTo(ArmPreset.CARGO);
            lastPickupPos = ArmPreset.CARGO;
            break;
        case HATCH:
            goTo(ArmPreset.PICKUP_HATCH);
            lastPickupPos = ArmPreset.PICKUP_HATCH;
            break;
        case EMPTY:
        default:
            if (lastPickupPos == ArmPreset.CARGO) {
                goTo(ArmPreset.PICKUP_HATCH);
                lastPickupPos = ArmPreset.PICKUP_HATCH;
            } else {
                goTo(ArmPreset.CARGO);
                lastPickupPos = ArmPreset.CARGO;
            }
            break;
        }
    }

    // Place arm in defense position
    public void stowArm() {
        goTo(ArmPreset.STOW);
    }

    // Move arm from defense position to lower deliver position.
    public void unStowArm() {
        goTo(ArmPreset.DELIVER_HATCH_LOWER);
    }

    public void goTo(int pos) {
        setArmMotionMagic(pos);
    }

    public void setArmMotionMagic(int pos) {
        armMaster.set(ControlMode.MotionMagic, pos);
    }

    public int getArmPos() {
        return armMaster.getSelectedSensorPosition(0);
    }

    public ArmPreset getCurrentArmPreset() {
        return currentArmPreset;
    }

    public void setCurrentArmPreset(ArmPreset preset) {
        currentArmPreset = preset;
    }

    public void setArmSoftLimits(int reverseSoftLimit, int forwardSoftLimit) {
        armMaster.configReverseSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        armMaster.configReverseSoftLimitThreshold(reverseSoftLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);

        armMaster.configForwardSoftLimitEnable(true, RobotMap.CTRE_TIMEOUT_PERIODIC);
        armMaster.configForwardSoftLimitThreshold(forwardSoftLimit, RobotMap.CTRE_TIMEOUT_PERIODIC);
    }

    public void setArmMotionProfile(int acceleration, int cruise) {
        armMaster.configMotionAcceleration(acceleration, RobotMap.CTRE_TIMEOUT_INIT);
        armMaster.configMotionCruiseVelocity(cruise, RobotMap.CTRE_TIMEOUT_INIT);
    }

    public void stop() {
        armMaster.set(ControlMode.PercentOutput, 0.0);
    }

    // Put items in here that you want updated on SmartDash during disableperiodic()
    public void updateSmartDash() {
        SmartDashboard.putNumber("Arm Pos", getArmPos());
        SmartDashboard.putNumber("WristAngle", currentArmPreset.getWristAngle());
        SmartDashboard.putString("Current Preset", currentArmPreset.toString());
        SmartDashboard.putString("Current State", curBotState.toString());
        SmartDashboard.putString("Last Lower Pos", lastLowerPos.toString());
        SmartDashboard.putString("Last Middle Pos", lastMiddlePos.toString());
        SmartDashboard.putString("Last Upper Pos", lastUpperPos.toString());
        SmartDashboard.putString("Last Pickup Pos", lastPickupPos.toString());
        SmartDashboard.putNumber("Shoulder Angle", currentArmPreset.getShoulderAngle());
        SmartDashboard.putNumber("Arm Lower Limit", reverseSoftLimit);
        SmartDashboard.putNumber("Arm Upper Limit", fwdSoftLimit);
    }

    @Override
    public void periodic() {
        updateSmartDash();
    }

    @Override
    protected void initDefaultCommand() {
        // There should be no default command for Jester Arm!!! Motion Magic PID control
        // handles it.
    }
}
