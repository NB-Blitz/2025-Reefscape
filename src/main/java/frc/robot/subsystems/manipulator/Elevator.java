package frc.robot.subsystems.manipulator;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator implements ElevatorInterface {

  // creating all the constants

  public static final int kLeftMotorCANID = 1;
  public static final int kRightMotorCANID = 2;

  public static final int kUpSwitchID = 3;

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

  public static final double kPositionConversionFactor = 0;
  public static final double kVelocityConversionFactor = 0;

  // the left motor is turning the opposite direction, the right motor is not
  public static final boolean kLeftInverted = true;
  public static final boolean kRightInverted = false;

  // create the motors, one on the left side of the elevator and one on the right side
  // we need two motors to provide enough power to move the elevator quickly
  private final SparkBase m_leftMotor = new SparkMax(kLeftMotorCANID, MotorType.kBrushless);
  private final SparkBase m_rightMotor = new SparkMax(kRightMotorCANID, MotorType.kBrushless);

  // create the relative encoders (one for each motor)
  // they track the position of the motors
  private final RelativeEncoder m_leftEncoder = m_leftMotor.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightMotor.getEncoder();

  // set the input for the elevator
  private final DigitalInput m_bottomSwitch = new DigitalInput(kUpSwitchID);

  // create the PID controller (only for the left motor)
  private final SparkClosedLoopController m_PIDController = m_leftMotor.getClosedLoopController();

  private final double maxElevatorSpeed = 1.0; // meters per second

  public Elevator() {

    // set up Spark Flex configuration for the left motor
    // (not sure what everything is set to but I think it works)
    var leftMotorConfig = new SparkFlexConfig();
    leftMotorConfig.inverted(kLeftInverted);
    leftMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(kMotorCurrentLimit)
        .voltageCompensation(12.0);
    leftMotorConfig
        .encoder
        .positionConversionFactor(kPositionConversionFactor)
        .velocityConversionFactor(kVelocityConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    leftMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(kP, kI, kD, kFF);
    leftMotorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    /*
     * leftMotorConfig
     * .limitSwitch
     * .forwardLimitSwitchEnabled(false)
     * .reverseLimitSwitchType(Type.kNormallyOpen)
     * .reverseLimitSwitchEnabled(true);
     * .softLimit
     * leftMotorConfig
     * .softLimit
    .forwardSoftLimit(0.0)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimit(0.0)
    .reverseSoftLimitEnabled(true)
     */

    // sets the configuration of the left motor
    tryUntilOk(
        m_leftMotor,
        5,
        () ->
            m_leftMotor.configure(
                leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // resets the left encoder position to 0.0
    tryUntilOk(m_leftMotor, 5, () -> m_leftEncoder.setPosition(0.0));

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
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(kP, kI, kD, kFF);
    rightMotorConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs(20)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    // rightMotorConfig.follow(m_rightMotor, true);

    // sets the configuration of the right motor
    tryUntilOk(
        m_rightMotor,
        5,
        () ->
            m_rightMotor.configure(
                rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // resets the right encoder position to 0.0
    tryUntilOk(m_rightMotor, 5, () -> m_rightEncoder.setPosition(0.0));
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
    return m_leftEncoder.getPosition();
  }
}
