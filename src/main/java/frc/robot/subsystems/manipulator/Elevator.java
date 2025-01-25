package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;

public class Elevator {

    public static final int leftElevatorMotorCANID = 1;
    public static final int rightElevatorMotorCANID = 2;

    public static final int kElevatorUpSwitchID = 3;

    public static final double kGearRatio = 1/12.0;
    public static final double kRotationSpeed = 1.0;
    public static final IdleMode kMotorIdleMode = IdleMode.kBrake;
    public static final int kMotorCurrentLimit = 39;

    public static final int kDownPosition = 0;
    public static final int kUpPosition = 1;

    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;



    private final SparkBase m_leftElevatorMotor = new SparkMax(leftElevatorMotorCANID, MotorType.kBrushless);
    private final SparkBase m_rightElevatorMotor = new SparkMax(rightElevatorMotorCANID, MotorType.kBrushless);

    private final RelativeEncoder m_leftElevatorEncoder = m_leftElevatorMotor.getEncoder();
    private final RelativeEncoder m_rightElevatorEncoder = m_rightElevatorMotor.getEncoder();

    private final DigitalInput m_elevatorUpSwitch = new DigitalInput(kElevatorUpSwitchID);

    private final SparkClosedLoopController m_elevatorPIDController = m_leftElevatorMotor.getClosedLoopController();

    private double leftPreviousDirection = -1;
    private double rightPreviousDirection = -1;

    public enum ElevatorPosition{
        coralL1(100),
        coralL2(200),
        coralL3(300),
        coralL4(400),
        algaeL1(250),
        algaeL2(350),
        coralIntake(250),
        algaeBarge(500),
        algaeProcessor(50),
        bottom(0),
        top(500);

        public final double position;

        ElevatorPosition(double position){
            this.position = position;
        }
    }

    public Elevator() {

        var elevatorConfig = new SparkFlexConfig();
        elevatorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(kMotorCurrentLimit)
            .voltageCompensation(12.0);
        elevatorConfig
            .encoder
            .positionConversionFactor(m_leftElevatorEncoder.getPositionConversionFactor())
            .velocityConversionFactor(m_leftElevatorEncoder.getVelocityConversionFactor())
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        elevatorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                0.1, 0.0,
                0.0, 0.0);
        elevatorConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        tryUntilOk(
            m_leftElevatorMotor,
            5,
            () ->
                m_leftElevatorMotor.configure(
                    elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(m_leftElevatorMotor, 5, () -> m_leftElevatorEncoder.setPosition(0.0));
    }


    public boolean getLimit() {
        return !m_elevatorUpSwitch.get();
    }


    public void moveToPosition(ElevatorPosition position) {
        m_elevatorPIDController.setReference(position.position, ControlType.kPosition);
    }

    public void move(double speed) {
        m_elevatorPIDController.setReference(speed, ControlType.kVelocity);
    }



}
