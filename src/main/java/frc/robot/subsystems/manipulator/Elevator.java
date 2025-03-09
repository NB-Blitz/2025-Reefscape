package frc.robot.subsystems.manipulator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator implements ElevatorInterface {

  // creating all the constants

  public static final int kLeadMotorCANID = 9;
  public static final int kFollowMotorCANID = 10;

  public static final double kWheelDiameter = 0.045; // in meters
  public static final double kWheelCircumference = Math.PI * kWheelDiameter; // in meters
  public static final double kGearRatio = 1 / 5.0;
  public static final double kRotationSpeed = 1.0;
  public static final IdleMode kMotorIdleMode = IdleMode.kBrake;
  public static final int kMotorCurrentLimit = 140;

  // these are values for the PID controller
  public static final double kP = 0.15;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kFF = 0.0;
  private double targetPosition = 0;
  private double targetSpeed = 0;
  private ControlType controlType = ControlType.kDutyCycle;

  public static final double kPositionConversionFactor =
      kGearRatio * kWheelCircumference; // in meters
  public static final double kVelocityConversionFactor =
      kPositionConversionFactor / 60.0; // in meters per second

  // the left motor is turning the opposite direction, the right motor is not
  public static final boolean kLeadInverted = false;

  // create the motors, one on the left side of the elevator and one on the right side
  // we need two motors to provide enough power to move the elevator quickly
  private final SparkBase m_followMotor = new SparkFlex(kFollowMotorCANID, MotorType.kBrushless);
  private final SparkBase m_leadMotor = new SparkFlex(kLeadMotorCANID, MotorType.kBrushless);

  // create the relative encoders (one for each motor)
  // they track the position of the motors
  private final RelativeEncoder m_leadEncoder = m_leadMotor.getEncoder();
  private final AbsoluteEncoder m_leadAbsEncoder = m_leadMotor.getAbsoluteEncoder();

  // set the input for the elevator
  private final SparkLimitSwitch m_bottomSwitch = m_leadMotor.getReverseLimitSwitch();
  // create the PID controller (only for the left motor)
  private final SparkClosedLoopController m_PIDController = m_leadMotor.getClosedLoopController();

  private final double maxElevatorSpeed = 2.5; // meters per second

  public Elevator() {

    // set up Spark Flex configuration for the left motor
    // (not sure what everything is set to but I think it works)
    var leadMotorConfig = new SparkFlexConfig();
    leadMotorConfig.inverted(kLeadInverted);
    leadMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kMotorCurrentLimit)
        .voltageCompensation(12.0);
    leadMotorConfig
        .encoder
        .positionConversionFactor(kPositionConversionFactor)
        .velocityConversionFactor(kVelocityConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    leadMotorConfig
        .absoluteEncoder
        .inverted(false)
        .positionConversionFactor(kPositionConversionFactor * 5)
        .velocityConversionFactor(kVelocityConversionFactor * 5)
        .averageDepth(2);
    leadMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(kP, kI, kD, kFF);
    leadMotorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs(10)
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    leadMotorConfig
        .limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true); // TODO Enable when limit switch is added
    leadMotorConfig
        .softLimit
        .forwardSoftLimit(0.73) // TODO update max height in meters
        .forwardSoftLimitEnabled(true);

    // sets the configuration of the right motor
    tryUntilOk(
        m_leadMotor,
        5,
        () ->
            m_leadMotor.configure(
                leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // resets the right encoder position to 0.0
    tryUntilOk(m_leadMotor, 5, () -> m_leadEncoder.setPosition(m_leadAbsEncoder.getPosition()));

    // set up Spark Flex configuration for the left motor
    // (not sure what everything is set to but I think it works)
    var followMotorConfig = new SparkFlexConfig();
    followMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kMotorCurrentLimit)
        .voltageCompensation(12.0);
    followMotorConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    // leftMotorConfig.follow(m_rightMotor, true);
    followMotorConfig.follow(kLeadMotorCANID, true);

    // sets the configuration of the left motor
    tryUntilOk(
        m_followMotor,
        5,
        () ->
            m_followMotor.configure(
                followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  // returns the state of the limit switch
  public boolean getLimit() {
    return m_bottomSwitch.isPressed();
  }

  // moves the elevator to a preset position specified by the Position parameter (created in the
  // enum)
  public void setPosition(ElevatorPosition position) {
    targetPosition = position.position;
    controlType = ControlType.kPosition;
  }

  public void setSpeed(double joystickInput) {
    targetSpeed = joystickInput * maxElevatorSpeed;
    controlType = ControlType.kVelocity;
  }

  // moves the elevator a certain speed according to the double parameter
  public void move() {
    if (getLimit()) {
      tryUntilOk(m_leadMotor, 5, () -> m_leadEncoder.setPosition(0));
    }
    double PIDTarget = targetSpeed;
    if (controlType == ControlType.kPosition) {
      PIDTarget = targetPosition;
    }
    m_PIDController.setReference(PIDTarget, controlType);
    SmartDashboard.putNumber("Elevator Height", getHeight());
    SmartDashboard.putNumber("Absolute Height", m_leadAbsEncoder.getPosition());
  }

  public double getHeight() {
    return m_leadEncoder.getPosition();
  }
}
