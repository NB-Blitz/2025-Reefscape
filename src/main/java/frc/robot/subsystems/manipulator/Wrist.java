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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class Wrist {

  // encoder - TBD gear ratio - TBD

  // ALL CAN IDs HIGHER THAN 12

  private double targetAngle = 0.0;
  private double targetSpeed = 0.0;
  private ControlType controlType;

  private SparkBase wristMotor;
  private DigitalInput wristLimitSwitch; // triggered = true
  private RelativeEncoder wristEncoder;
  private SparkClosedLoopController PIDController = wristMotor.getClosedLoopController();

  private final double wristP = 0.5;
  private final double wristI = 0.0;
  private final double wristD = 0.0;
  private final double wristFF = 0.0;
  private final double gearRatio = 12 / 38;
  private final double wristEncoderPositionFactor =
      2 * Math.PI / gearRatio; // Rotor Rotations -> Wheel Radians
  private final double wristEncoderVelocityFactor = (2 * Math.PI) / 60.0 / gearRatio;
  private final int currentLimit = 40;
  private final double wristAngleMax = 120.0;
  private final int wristMotorCANID = 314159; // TODO: fix the can ids
  private final int wristLimitSwitchCANID = 271817181;
  private final double maxWristSpeed = 90; // degrees per second

  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public static enum WristAngle {
    coralBottom(-1),
    coralMiddle(-1),
    coralTop(-1),
    coralIntake(-1),
    algaeInReef(-1),
    algaeBarge(-1),
    algaeProcessor(-1),
    algaeIntake(-1),
    bottom(-1),
    top(-1);

    public final double angle;

    WristAngle(double angle) {
      this.angle = angle;
    }
  }

  public Wrist() {
    wristMotor = new SparkMax(wristMotorCANID, MotorType.kBrushless);
    wristLimitSwitch = new DigitalInput(wristLimitSwitchCANID);
    wristEncoder = wristMotor.getEncoder();

    var driveConfig =
        new SparkFlexConfig(); // TODO: invert the motor if needed to make positive motor speed
    // increase the angle(go down) and negative motor speed decrease the
    // angle (go up)
    driveConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(currentLimit).voltageCompensation(12.0);
    driveConfig
        .encoder
        .positionConversionFactor(wristEncoderPositionFactor)
        .velocityConversionFactor(wristEncoderVelocityFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    driveConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            wristP, wristI,
            wristD, wristFF);
    driveConfig
        .signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderPositionPeriodMs((int) (1000.0 / 100.0))
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    tryUntilOk(
        wristMotor,
        5,
        () ->
            wristMotor.configure(
                driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(wristMotor, 5, () -> wristEncoder.setPosition(0.0));
    wristEncoder.setPosition(0.0);
  }

  public void resetEncoder(double angle) {
    wristEncoder.setPosition(angle);
  }

  public double getPosition() {
    return wristEncoder.getPosition();
  }

  public boolean getLimitSwitch() {
    return wristLimitSwitch.get();
  }

  public void setWristSpeed(double joystickInput) {
    targetSpeed = joystickInput * maxWristSpeed;
    controlType = ControlType.kVelocity;
  }

  public void setWristAngle(WristAngle angle) {
    targetAngle = angle.angle;
    controlType = ControlType.kPosition;
  }

  public void updateWrist() {
    /*
    if(wristLimitSwitch.get()){
        resetEncoder(0.0);
    }
    */
    double PIDTarget = targetSpeed;
    if (controlType == ControlType.kPosition) PIDTarget = targetAngle;
    PIDController.setReference(PIDTarget, controlType);
  }
}
