/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team2363.logger.HelixLogger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Auto.undockJester;
import frc.robot.ArmPreset;

public class JesterArm extends Subsystem {

    private TalonSRX armMaster = new TalonSRX(RobotMap.ARM_MASTER_ID);
    private VictorSPX armSlave = new VictorSPX(RobotMap.ARM_SLAVE_ID);

    public static int ARM_ACCELERATION = 30;
    public static int ARM_CRUISE = 3;

    private static JesterArm INSTANCE = new JesterArm();
    // private ArmPreset currentArmPreset = ArmPreset.START;
    private ArmPreset currentArmPreset = ArmPreset.START;
    // private ArmPreset currentArmPreset = null;


    private int reverseSoftLimit, fwdSoftLimit;
    // private static DigitalInput botState = new DigitalInput(1);

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

        currentArmPreset = ArmPreset.START;
        // currentArmPreset = ArmPreset.UNPACK_WP1;


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

        reverseSoftLimit = ArmPreset.PICKUP_HATCH.CalculateArmPos();  // lower sensor reading side
        fwdSoftLimit = ArmPreset.DELIVER_HATCH_LOWER.CalculateArmPos();   // higher sensor reading side
        setArmSoftLimits(reverseSoftLimit, fwdSoftLimit);
        setArmMotionProfile(ARM_ACCELERATION, ARM_CRUISE);

        // The arm starts the match in a one-time docked position. Move arm from
        // docked position to front lower scoring position.
        // boolean compMode = botState.get();
        // boolean pitMode = true;
        // unDockArm();
    }

    private void setupLogs() {
        HelixLogger.getInstance().addDoubleSource("ARM MASTER CURRENT", armMaster::getOutputCurrent);
        // HelixLogger.getInstance().addDoubleSource("ARM SLAVE", () -> pdp.getCurrent(RobotMap.ARM_SLAVE_ID));
    }

    public void goTo(ArmPreset preset) {
        int newPos = preset.CalculateArmPos();
        currentArmPreset = preset;
        SmartDashboard.putString("goTo: ", preset.toString());
        SmartDashboard.putNumber("Calc Arm Pos", newPos);
        setArmMotionMagic(newPos);
    }

    public void Up() {
        switch (currentArmPreset) {
            case DELIVER_HATCH_LOWER:
                goTo(ArmPreset.DELIVER_BALL_LOWER);
                break;
            case DELIVER_BALL_LOWER:
                goTo(ArmPreset.DELIVER_HATCH_MIDDLE);
                break;
            case DELIVER_HATCH_MIDDLE:
                goTo(ArmPreset.DELIVER_BALL_MIDDLE);
                break;
            case DELIVER_BALL_MIDDLE:
                goTo(ArmPreset.DELIVER_HATCH_UPPER);
                break;
            case DELIVER_HATCH_UPPER:
                goTo(ArmPreset.DELIVER_BALL_UPPER);
                break;
            case DELIVER_BALL_UPPER:  // At Top Deliver Side
            case CARGO:               // At Top Pickup Side
                // At top
                break;
            case PICKUP_CARGO_FLOOR:
                goTo(ArmPreset.CARGO);
                break;
            case PICKUP_HATCH:
                goTo(ArmPreset.PICKUP_CARGO_FLOOR);
                break;
            default:    
                break;
        }
    }   

    public void Down() {
        switch (currentArmPreset) {
            case DELIVER_HATCH_LOWER:
            case PICKUP_HATCH:
                // At Bottom
                break;
            case DELIVER_BALL_LOWER:
                goTo(ArmPreset.DELIVER_HATCH_LOWER);
                break;
            case DELIVER_HATCH_MIDDLE:
                goTo(ArmPreset.DELIVER_BALL_LOWER);
                break;
            case DELIVER_BALL_MIDDLE:
                goTo(ArmPreset.DELIVER_HATCH_MIDDLE);
                break;
            case DELIVER_HATCH_UPPER:
                goTo(ArmPreset.DELIVER_BALL_MIDDLE);
                break;
            case DELIVER_BALL_UPPER:
                goTo(ArmPreset.DELIVER_HATCH_UPPER);
                break;
            case CARGO:       
                goTo(ArmPreset.PICKUP_CARGO_FLOOR);
                break;
            case PICKUP_CARGO_FLOOR:
                goTo(ArmPreset.PICKUP_HATCH);
                break;
            default:    
                break;
        }
    } 

    public void toggleArm() {
        switch (currentArmPreset) {
        case CARGO:
            goTo(ArmPreset.DELIVER_BALL_UPPER);
            break;
        case DELIVER_BALL_UPPER:
            goTo(ArmPreset.CARGO);
            break;
        case DELIVER_HATCH_UPPER:
            goTo(ArmPreset.CARGO);
            break;
        default:
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

    // Move arm from docked position (at start of match) to front lower scoring
    // position.
    public void unDockArm() {
        // goTo(Arm);
        // new undockJester();
        // goTo(ArmPreset.UNPACK_WP1);
        // goTo(ArmPreset.UNPACK_WP1);
        // new WaitCommand(1000);
        // try {
        //     wait(10000);
        // }catch (Exception e) {}
        goTo(ArmPreset.UNPACK_WP3);
        
        // goTo(ArmPreset.DELIVER_HATCH_LOWER);
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
        //  There should be no default command for Jester Arm!!! Motion Magic PID control
        //  handles it.
    }
}
