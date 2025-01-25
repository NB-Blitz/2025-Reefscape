package frc.robot.subsystems.manipulator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;

public class Hand {

    private final int leftMotorCANID = 13;
    private final int rightMotorCANID = 14;
    private final int coralFrontSwitchOID = 1;
    private final int coralBackSwitchOID = 2; 
    private final int algaeSwitchOID = 3;
    private final int currentLimit = 40;

    private final double coralIntakeSpeed = 0.1;
    private final double coralExpelSpeed = 0.1;
    private final double algaeIntakeSpeed = 0.1;
    private final double algaeExpelHighSpeed = -0.1;
    private final double algaeExpelLowSpeed = -0.1;

    private final boolean motorInverted = true;

    private final SparkBase leftMotor;
    private final SparkBase rightMotor;

    //true when pressed
    private final DigitalInput coralFrontSwitch;
    private final DigitalInput coralBackSwitch;
    //true when pressed
    private final DigitalInput algaeSwitch;

    public Hand() {

        leftMotor = new SparkMax(leftMotorCANID, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorCANID, MotorType.kBrushless);

        coralFrontSwitch = new DigitalInput(coralFrontSwitchOID);
        coralBackSwitch = new DigitalInput(coralBackSwitchOID);
        algaeSwitch = new DigitalInput(algaeSwitchOID);

        var rightMotorConfig = new SparkMaxConfig();
        rightMotorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(motorInverted)
            .smartCurrentLimit(currentLimit)
            .voltageCompensation(12.0);
        rightMotorConfig
            .signals
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        tryUntilOk(
            rightMotor,
            5,
            () ->
                rightMotor.configure(
                    rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        var leftMotorConfig = new SparkMaxConfig();
        leftMotorConfig
            .idleMode(IdleMode.kBrake)
            .inverted(!motorInverted)
            .smartCurrentLimit(currentLimit)
            .voltageCompensation(12.0);
        leftMotorConfig
            .signals
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        tryUntilOk(
            leftMotor,
            5,
            () ->
                leftMotor.configure(
                    leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public void intakeCoral() {
        if(coralBackSwitch.get() == false) {
            leftMotor.set(coralIntakeSpeed);
            rightMotor.set(coralIntakeSpeed);
        }
        else {
            leftMotor.set(0);
            rightMotor.set(0);
        }
    }

    public void expelCoral() {
        leftMotor.set(coralExpelSpeed);
        rightMotor.set(coralExpelSpeed);
        if(coralBackSwitch.get() == false && coralFrontSwitch.get() == false) {
            leftMotor.set(0);
            rightMotor.set(0);
        }
    }

    public void stopMotors() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public boolean coralInPosition() {
        if(coralFrontSwitch.get() == true && coralBackSwitch.get() == false) {
            return true;
        }
        return false;
    }

    public void intakeAlgae() {
        if(algaeSwitch.get() == false && coralBackSwitch.get() == false && coralFrontSwitch.get() == false) {
            leftMotor.set(algaeIntakeSpeed);
            rightMotor.set(algaeIntakeSpeed);
        }
        else {
            leftMotor.set(0);
            rightMotor.set(0);
        }
    }

    public void expelAlgaeHigh() {
        leftMotor.set(algaeExpelHighSpeed);
        rightMotor.set(algaeExpelHighSpeed);
    }

    public void expelAlgaeLow() {
        leftMotor.set(algaeExpelLowSpeed);
        rightMotor.set(algaeExpelLowSpeed);
    }

    public boolean algaeInPosition() {
        return algaeSwitch.get();
    }
    
}