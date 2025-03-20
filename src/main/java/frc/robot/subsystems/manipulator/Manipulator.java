package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.common.Joint;
import frc.robot.subsystems.manipulator.ElevatorInterface.ElevatorPosition;
import frc.robot.subsystems.manipulator.Shoulder.ShoulderAngle;
import frc.robot.subsystems.manipulator.Wrist.WristAngle;
import frc.robot.util.LEDStrip;

public class Manipulator extends SubsystemBase {
  // Set to "Blank" versions to disable each component
  private final ElevatorInterface elevator = new Elevator();
  private final Joint shoulder = new Shoulder();
  private final Joint wrist = new Wrist();
  private final HandInterface hand = new Hand();
  private final LEDStrip ledStrip = new LEDStrip(0, 58);

  // private final double elevatorHeightTolerance = 0.025; // meters
  private final double shoulderNoFoulTolerance = 28.0; // degrees
  private int cachedWristIndex = 0;

  private final String[] presetNames = {
    "Bottom",
    "Processor",
    "Coral L1",
    "Coral Station",
    "Coral L2",
    "Algae L2",
    "Coral L3",
    "Algae L3",
    "Coral L4",
    "Barge"
  };

  private final ElevatorPosition[] elevatorPositions = {
    ElevatorPosition.bottom,
    ElevatorPosition.algaeProcessor,
    ElevatorPosition.coralL1,
    ElevatorPosition.coralIntake,
    ElevatorPosition.coralL2,
    ElevatorPosition.algaeInReefL2,
    ElevatorPosition.coralL3,
    ElevatorPosition.algaeInReefL3,
    ElevatorPosition.coralL4,
    ElevatorPosition.algaeBarge
  };

  private final ShoulderAngle[] shoulderAngles = {
    ShoulderAngle.bottom,
    ShoulderAngle.algaeProcessor,
    ShoulderAngle.coralL1,
    ShoulderAngle.coralIntake,
    ShoulderAngle.coralL2,
    ShoulderAngle.algaeInReefL2,
    ShoulderAngle.coralL3,
    ShoulderAngle.algaeInReefL3,
    ShoulderAngle.coralL4,
    ShoulderAngle.algaeBarge
  };

  private final WristAngle[] wristAngles = {
    WristAngle.bottom,
    WristAngle.algaeProcessor,
    WristAngle.coralL1,
    WristAngle.coralIntake,
    WristAngle.coralL2,
    WristAngle.algaeInReefL2,
    WristAngle.coralL3,
    WristAngle.algaeInReefL3,
    WristAngle.coralL4,
    WristAngle.algaeBarge
  };

  // 0-11
  private int levelIndex = 0;
  private boolean positionCommand = false;
  private boolean joystickCommand = true;
  private boolean isIntakingCoral = false;
  private boolean isExpellingCoral = false;

  public void periodic() {
    /**
     * Any automatic behavior we want An example would be if we have a note and are in the top
     * position we could start the shooting motors.
     */
    elevator.move();
    shoulder.updateJoint();
    wrist.updateJoint();

    // SmartDashboard.putString("Preset Level", presetNames[levelIndex]);
    String[] display = presetNames.clone();
    if (positionCommand) {
      display[levelIndex] = ">>> " + display[levelIndex];
    }
    SmartDashboard.putStringArray("Preset Level", display);

    double ledRatio = elevator.getHeight() / elevator.getTopLimit();
    ledStrip.updateLEDs(hand.holdingCoral(), ledRatio);
  }

  public void incrementLevel() {
    if (joystickCommand) {
      double smallestDiff = Integer.MAX_VALUE;
      double height = elevator.getHeight();
      for (int i = 1; i < elevatorPositions.length; i++) {
        double diff = elevatorPositions[i].position - height;
        if (diff > 0 && Math.abs(diff) < smallestDiff) {
          levelIndex = i;
          smallestDiff = diff;
        }
      }
      joystickCommand = false;
    } else {
      if (levelIndex < elevatorPositions.length - 1) levelIndex++;
    }
    positionCommand = true;
  }

  public void decrementLevel() {
    if (joystickCommand) {
      double smallestDiff = Integer.MAX_VALUE;
      double height = elevator.getHeight();
      for (int i = elevatorPositions.length - 2; i >= 0; i--) {
        double diff = elevatorPositions[i].position - height;
        if (diff < 0 && Math.abs(diff) < smallestDiff) {
          levelIndex = i;
          smallestDiff = diff;
        }
      }
      joystickCommand = false;
    } else {
      if (levelIndex > 0) levelIndex--;
    }
    positionCommand = true;
  }

  public void intakeCoralExpelAlgae() {
    isIntakingCoral = true;
    if (isExpellingCoral) {
      hand.stopMotors();
    } else {
      if (hand.holdingCoral() && positionCommand && levelIndex != 8) {
        hand.stopMotors();
      } else {
        hand.intakeCoral();
      }
    }
  }

  public void expelCoralIntakeAlgae() {
    isExpellingCoral = true;
    if (isIntakingCoral) {
      hand.stopMotors();
    } else {
      if (hand.holdingCoral() && positionCommand && levelIndex == 8) {
        hand.stopMotors();
      } else {
        hand.expelCoral();
      }
    }
  }

  public void stopIntakeCoral() {
    isIntakingCoral = false;
    if (!isExpellingCoral) {
      hand.stopMotors();
    }
  }

  public void stopExpelCoral() {
    isExpellingCoral = false;
    if (!isIntakingCoral) {
      hand.stopMotors();
    }
  }

  public void emergencyStop() {
    elevator.eStop();
    shoulder.eStop();
    wrist.eStop();
    hand.stopMotors();
  }

  public void runManipulator(double triggers, double leftJoy, double rightJoy) {
    // if we recieve a joystick input stop both auto positioning
    if (triggers == 0.0 && leftJoy == 0.0 && rightJoy == 0.0 && positionCommand) {
      elevator.setPosition(elevatorPositions[levelIndex]);
      cachedWristIndex = levelIndex;
      wrist.setJointAngle(wristAngles[levelIndex].ordinal());
      shoulder.setJointAngle(shoulderAngles[levelIndex].ordinal());
    } else if (joystickCommand || triggers != 0.0 || leftJoy != 0.0 || rightJoy != 0.0) {
      elevator.setSpeed(triggers);
      wrist.setJointSpeed(rightJoy);
      shoulder.setJointSpeed(leftJoy);
      joystickCommand = true;
    }
    positionCommand = false;

    if (shoulder.getPosition() > shoulder.noFoulPos - shoulderNoFoulTolerance
        && shoulder.getPosition() < shoulder.noFoulPos + shoulderNoFoulTolerance) {
      double angleDiff = shoulder.noFoulPos - shoulder.getPosition();
      wrist.setJointAngleRaw(wrist.noFoulPos + angleDiff);
    } else if (!joystickCommand) {
      wrist.setJointAngle(wristAngles[cachedWristIndex].ordinal());
    }
  }

  public void resetTargets() {
    elevator.resetTargetHeight();
    shoulder.resetTargetAngle();
    wrist.resetTargetAngle();
  }
}
