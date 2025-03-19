package frc.robot.subsystems.manipulator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class Hand implements HandInterface {

  private final int leftMotorCANID = 13;
  private final int rightMotorCANID = 14;
  private final int coralFrontSwitchOID = 1;
  private final int coralBackSwitchOID = 2;
  private final int algaeSwitchOID = 3;
  private final int currentLimit = 80;

  // TODO update speeds
  private final double coralIntakeSpeed = 0.5;
  private final double coralExpelSpeed = 0.5;
  private final double algaeIntakeSpeed = 0.5;
  private final double algaeExpelNetSpeed = -0.5;
  private final double algaeExpelProcessorSpeed = -0.5;

  private final boolean motorInverted = true;

  private final SparkBase leftMotor;
  private final SparkBase rightMotor;

  // true when pressed
  private final DigitalInput coralFrontSwitch;
  private final DigitalInput coralBackSwitch;
  // true when pressed
  private final DigitalInput algaeSwitch;

  // private final double p = 0.0001;
  // private final double RPM = 60 * 5;
  private final double percent = 0.5;

  // private SparkClosedLoopController righthandController;
  // private SparkClosedLoopController lefthandController;

  public Hand() {

    leftMotor = new SparkFlex(leftMotorCANID, MotorType.kBrushless);
    rightMotor = new SparkFlex(rightMotorCANID, MotorType.kBrushless);
    // righthandController = rightMotor.getClosedLoopController();
    // lefthandController = leftMotor.getClosedLoopController();

    coralFrontSwitch = new DigitalInput(coralFrontSwitchOID);
    coralBackSwitch = new DigitalInput(coralBackSwitchOID);
    algaeSwitch = new DigitalInput(algaeSwitchOID);

    SparkFlexConfig rightMotorConfig = new SparkFlexConfig();
    rightMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(motorInverted)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    // rightMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(p, 0, 0, 0);
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

    SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
    leftMotorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(!motorInverted)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0);
    // leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(p, 0, 0, 0);
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
    if (coralBackSwitch.get() == false && algaeInPosition() == false) {
      leftMotor.set(coralIntakeSpeed);
      rightMotor.set(coralIntakeSpeed);
    } else {
      leftMotor.set(0);
      rightMotor.set(0);
    }
  }

  public void clockwise() {
    leftMotor.set(percent);
    rightMotor.set(percent);
    // lefthandController.setReference(RPM, ControlType.kVelocity);
    // righthandController.setReference(RPM, ControlType.kVelocity);
  }

  public void counterClockwise() {
    leftMotor.set(-percent);
    rightMotor.set(-percent);
    // lefthandController.setReference(-RPM, ControlType.kVelocity);
    // righthandController.setReference(-RPM, ControlType.kVelocity);
  }

  public void expelCoral() {
    leftMotor.set(coralExpelSpeed);
    rightMotor.set(coralExpelSpeed);
    if (coralBackSwitch.get() == false && coralFrontSwitch.get() == false) {
      leftMotor.set(0);
      rightMotor.set(0);
    }
  }

  public void stopMotors() {
    leftMotor.set(0);
    rightMotor.set(0);
    // lefthandController.setReference(0, ControlType.kVelocity);
    // righthandController.setReference(0, ControlType.kVelocity);
  }

  public boolean coralInPosition() {
    if (coralFrontSwitch.get() == true && coralBackSwitch.get() == false) {
      return true;
    }
    return false;
  }

  public void intakeAlgae() {
    if (algaeSwitch.get() == false
        && coralBackSwitch.get() == false
        && coralFrontSwitch.get() == false) {
      leftMotor.set(algaeIntakeSpeed);
      rightMotor.set(algaeIntakeSpeed);
    } else {
      leftMotor.set(0);
      rightMotor.set(0);
    }
  }

  public void expelAlgaeNet() {
    leftMotor.set(algaeExpelNetSpeed);
    rightMotor.set(algaeExpelNetSpeed);
  }

  public void expelAlgaeProcessor() {
    leftMotor.set(algaeExpelProcessorSpeed);
    rightMotor.set(algaeExpelProcessorSpeed);
  }

  public boolean algaeInPosition() {
    return algaeSwitch.get();
  }
}
