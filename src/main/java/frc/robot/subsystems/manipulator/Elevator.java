package frc.robot.subsystems.manipulator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator implements ElevatorInterface {

  // creating all the constants

  public static final int kLeftMotorCANID = 1;
  public static final int kRightMotorCANID = 2;

  public static final int kUpSwitchID = 3;

  public static final double kWheelDiameter = 0.04; // in meters
  public static final double kWheelCircumference = Math.PI * kWheelDiameter; // in meters
  public static final double kGearRatio = 1 / 5.0;
  public static final double kRotationSpeed = 1.0;
  public static final IdleMode kMotorIdleMode = IdleMode.kBrake;
  public static final int kMotorCurrentLimit = 39;

  public static final int kDownPosition = 0;
  public static final int kUpPosition = 1;

  // these are values for the PID controller
  public static final double kP = 0.1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kFF = 0.0;
  private double targetPosition = 0;
  private double targetSpeed = 0;
  private ControlType controlType = ControlType.kDutyCycle;

  public static final double kPositionConversionFactor =
      kGearRatio * kWheelCircumference * 2.0; // in meters
  public static final double kVelocityConversionFactor =
      kPositionConversionFactor / 60.0; // in meters per second TODO check this

  // the left motor is turning the opposite direction, the right motor is not
  public static final boolean kRightInverted = true;

  // create the motors, one on the left side of the elevator and one on the right side
  // we need two motors to provide enough power to move the elevator quickly
  private final SparkBase m_leftMotor = new SparkMax(kLeftMotorCANID, MotorType.kBrushless);
  private final SparkBase m_rightMotor = new SparkMax(kRightMotorCANID, MotorType.kBrushless);

  // create the relative encoders (one for each motor)
  // they track the position of the motors
  private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();
  private final AbsoluteEncoder m_rightAbsEncoder = m_rightMotor.getAbsoluteEncoder();

  // set the input for the elevator
  private final DigitalInput m_bottomSwitch = new DigitalInput(kUpSwitchID);

  // create the PID controller (only for the left motor)
  private final SparkClosedLoopController m_PIDController = m_leftMotor.getClosedLoopController();

  private final double maxElevatorSpeed = 0.2; // meters per second

  public Elevator() {

    // set up Spark Flex configuration for the left motor
    // (not sure what everything is set to but I think it works)
    var rightMotorConfig = new SparkFlexConfig();
    rightMotorConfig.inverted(kRightInverted);
    rightMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kMotorCurrentLimit)
        .voltageCompensation(12.0);
    rightMotorConfig
        .encoder
        .positionConversionFactor(kPositionConversionFactor)
        .velocityConversionFactor(kVelocityConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    rightMotorConfig
        .absoluteEncoder
        .inverted(true) // TODO check this
        .positionConversionFactor(kPositionConversionFactor * 5)
        .velocityConversionFactor(kVelocityConversionFactor * 5)
        .averageDepth(2);
    rightMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP, kI, kD, kFF);
    rightMotorConfig
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
    rightMotorConfig
        .limitSwitch
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false); //TODO Enable when limit switch is added
    rightMotorConfig
        .softLimit
        .forwardSoftLimit(1) // TODO update max height in meters
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(0.0)
        .reverseSoftLimitEnabled(true);

    // sets the configuration of the right motor
    tryUntilOk(
        m_rightMotor,
        5,
        () ->
            m_rightMotor.configure(
                rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // resets the right encoder position to 0.0
    tryUntilOk(m_rightMotor, 5, () -> m_rightEncoder.setPosition(m_rightAbsEncoder.getPosition()));

    // set up Spark Flex configuration for the left motor
    // (not sure what everything is set to but I think it works)
    var leftMotorConfig = new SparkFlexConfig();
    leftMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kMotorCurrentLimit)
        .voltageCompensation(12.0);
    leftMotorConfig
        .signals
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    leftMotorConfig.follow(m_rightMotor, true);

    // sets the configuration of the left motor
    tryUntilOk(
        m_leftMotor,
        5,
        () ->
            m_leftMotor.configure(
                leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  // returns the state of the limit switch
  public boolean getLimit() {
    return m_bottomSwitch.get();
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
    double PIDTarget = targetSpeed;
    if (controlType == ControlType.kPosition) {
      PIDTarget = targetPosition;
    }
    m_PIDController.setReference(PIDTarget, controlType);
  }

  public double getHeight() {
    return m_rightEncoder.getPosition();
  }
}
