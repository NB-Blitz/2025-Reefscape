package frc.robot.subsystems.common;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Joint {

  protected double targetAngle = 0.0;
  protected double targetSpeed = 0.0;
  protected ControlType controlType = ControlType.kDutyCycle;

  protected SparkBase jointMotor;
  private RelativeEncoder jointEncoder;
  protected AbsoluteEncoder jointAbsoluteEncoder;
  private SparkClosedLoopController jointController;
  private final int currentLimit = 200;
  private final double maxJointSpeed;
  protected final double angleOffset;
  private final double kAngleTolerance =
      2.0; // If the relaive encoder is plus or minus this value it sets it to the absolute tl;dr
  // fixes belt skipping

  public Joint(
      double P,
      double I,
      double D,
      double FF,
      double gearRatio,
      double maxSpeed,
      double absGearRatio,
      boolean absInvert,
      boolean invert,
      double kForwardSoftLimit,
      double kReverseSoftLimit,
      double angleOffsettywettyfetty,
      SparkBase motorRef,
      SparkBaseConfig config) {
    jointMotor = motorRef;
    jointController = jointMotor.getClosedLoopController();
    this.maxJointSpeed = maxSpeed;
    angleOffset = angleOffsettywettyfetty;

    jointEncoder = jointMotor.getEncoder();
    jointAbsoluteEncoder = jointMotor.getAbsoluteEncoder();

    var jointConfig = config;

    jointConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    jointConfig.inverted(invert);
    jointConfig
        .encoder
        .positionConversionFactor(gearRatio * 360)
        .velocityConversionFactor(gearRatio * 6)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    jointConfig
        .absoluteEncoder
        .inverted(absInvert)
        .positionConversionFactor(absGearRatio * 360)
        .velocityConversionFactor(absGearRatio * 6)
        .averageDepth(2);
    jointConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            P, I,
            D, FF)
        .positionWrappingEnabled(true);
    jointConfig
        .softLimit
        .forwardSoftLimit(kForwardSoftLimit + angleOffset)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(kReverseSoftLimit + angleOffset)
        .reverseSoftLimitEnabled(true);
    jointConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(10)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        jointMotor,
        5,
        () ->
            jointMotor.configure(
                jointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(jointMotor, 5, () -> jointEncoder.setPosition(jointAbsoluteEncoder.getPosition()));
  }

  public Joint(
      double P,
      double I,
      double D,
      double FF,
      double gearRatio,
      double maxSpeed,
      boolean invert,
      SparkBase motorRef,
      SparkBaseConfig config) {
    angleOffset = 0.0;
    jointMotor = motorRef;
    jointController = jointMotor.getClosedLoopController();
    this.maxJointSpeed = maxSpeed;
    jointEncoder = jointMotor.getEncoder();
    jointAbsoluteEncoder = null;

    var jointConfig = config;
    jointConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    jointConfig.inverted(invert);
    jointConfig
        .encoder
        .positionConversionFactor(gearRatio * 360)
        .velocityConversionFactor(gearRatio * 6)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    jointConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            P, I,
            D, FF)
        .positionWrappingEnabled(true);
    jointConfig
        .softLimit
        .forwardSoftLimit(180) // TODO update max height in meters
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true);
    jointConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(10)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        jointMotor,
        5,
        () ->
            jointMotor.configure(
                jointConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(jointMotor, 5, () -> jointEncoder.setPosition(0));
  }

  public Joint() {
    angleOffset = 0.0;
    maxJointSpeed = 0;
  }

  public void resetEncoder(double angle) {
    jointEncoder.setPosition(angle + angleOffset);
  }

  public double getPosition() {
    return jointEncoder.getPosition() - angleOffset;
  }

  public void setJointSpeed(double joystickInput) {
    targetSpeed = joystickInput * maxJointSpeed;
    controlType = ControlType.kVelocity;
    if (targetSpeed == 0) {
      targetAngle = jointEncoder.getPosition();
      controlType = ControlType.kPosition;
    }
  }

  public void setJointAngle(int enumIndex) {
    targetAngle = enumIndex + angleOffset;
    controlType = ControlType.kPosition;
  }

  public void updateJoint() {
    if (jointAbsoluteEncoder != null) {
      if (Math.abs(jointAbsoluteEncoder.getPosition() - jointEncoder.getPosition())
          > kAngleTolerance) {
        tryUntilOk(
            jointMotor, 5, () -> jointEncoder.setPosition(jointAbsoluteEncoder.getPosition()));
      }
    }
    double PIDTarget = targetSpeed;
    if (controlType == ControlType.kPosition) PIDTarget = targetAngle;
    jointController.setReference(PIDTarget, controlType);
  }
}
