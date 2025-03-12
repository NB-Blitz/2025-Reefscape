package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.common.Joint;
import frc.robot.subsystems.manipulator.ElevatorInterface.ElevatorPosition;
import frc.robot.subsystems.manipulator.Shoulder.ShoulderAngle;
import frc.robot.subsystems.manipulator.Wrist.WristAngle;

public class Manipulator extends SubsystemBase {
  // Set to "Blank" versions to disable each component
  private final HandInterface robotHand = new Hand();
  private final Joint robotWrist = new Wrist();
  private final ElevatorInterface robotElevator = new ElevatorBlank();
  private final double elevatorHeightTolerance = 0.03;
  private final Joint robotShoulder = new Shoulder();

  private final ElevatorPosition[] elevatorPositions = {
    ElevatorPosition.bottom,
    ElevatorPosition.algaeIntake,
    ElevatorPosition.algaeProcessor,
    ElevatorPosition.coralL1,
    ElevatorPosition.coralL2,
    ElevatorPosition.algaeL1,
    ElevatorPosition.coralIntake,
    ElevatorPosition.coralL3,
    ElevatorPosition.algaeL2,
    ElevatorPosition.coralL4,
    ElevatorPosition.algaeBarge,
    ElevatorPosition.top
  };

  private final WristAngle[] wristAngles = {
    WristAngle.coralBottom,
    WristAngle.algaeIntake,
    WristAngle.algaeProcessor,
    WristAngle.coralBottom,
    WristAngle.coralMiddle,
    WristAngle.algaeInReef,
    WristAngle.coralIntake,
    WristAngle.coralMiddle,
    WristAngle.algaeInReef,
    WristAngle.coralTop,
    WristAngle.algaeBarge,
    WristAngle.algaeBarge
  };

  private final ShoulderAngle[] shoulderAngles = {
    ShoulderAngle.coralBottom,
    ShoulderAngle.algaeIntake,
    ShoulderAngle.algaeProcessor,
    ShoulderAngle.coralBottom,
    ShoulderAngle.coralMiddle,
    ShoulderAngle.algaeInReef,
    ShoulderAngle.coralIntake,
    ShoulderAngle.coralMiddle,
    ShoulderAngle.algaeInReef,
    ShoulderAngle.coralTop,
    ShoulderAngle.algaeBarge,
    ShoulderAngle.algaeBarge
  };

  // 0-11
  private int levelIndex = 0;
  private boolean positionCommand = false;
  private boolean joystickCommand = false;
  private boolean isIntaking = false;
  private boolean isExpelling = false;

  public void periodic() {
    /**
     * Any automatic behavior we want An example would be if we have a note and are in the top
     * position we could start the shooting motors.
     */
    robotElevator.move();
    robotShoulder.updateJoint();
    robotWrist.updateJoint();
  }

  public void incrementLevel() {
    if (joystickCommand) {
      double height = robotElevator.getHeight();
      for (int i = 1; i < elevatorPositions.length; i++) {
        if (height < elevatorPositions[i].position - elevatorHeightTolerance) {
          levelIndex = i;
          break;
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
      double height = robotElevator.getHeight();
      for (int i = elevatorPositions.length - 2; i >= 0; i--) {
        if (height > elevatorPositions[i].position + elevatorHeightTolerance) {
          levelIndex = i;
          break;
        }
      }
      joystickCommand = false;
    } else {
      if (levelIndex > 0) levelIndex--;
    }
    positionCommand = true;
  }

  public void expel() {
    if (!isExpelling) {
      robotHand.clockwise();
      isIntaking = false;
    } else {
      robotHand.stopMotors();
    }
    isExpelling = !isExpelling;
  }

  public void intake() {
    if (!isIntaking) {
      robotHand.counterClockwise();
      isExpelling = false;
    } else {
      robotHand.stopMotors();
    }
    isIntaking = !isIntaking;
  }

  public void stopHand() {
    robotHand.stopMotors();
    isExpelling = false;
    isIntaking = false;
  }

  public void emergencyStop() {
    robotElevator.setSpeed(0.0);
    robotWrist.setJointSpeed(0.0);
    robotHand.stopMotors();
    robotShoulder.setJointSpeed(0.0);
  }

  public void runManipulator(double triggers, double leftJoy, double rightJoy) { // TODO: test this

    // if we recieve a joystick input stop both auto positioning
    if (triggers == 0.0 && leftJoy == 0.0 && rightJoy == 0.0 && positionCommand) {
      robotElevator.setPosition(elevatorPositions[levelIndex]);
      robotWrist.setJointAngle(wristAngles[levelIndex].ordinal());
      robotShoulder.setJointAngle(shoulderAngles[levelIndex].ordinal());
    } else if (joystickCommand || triggers != 0.0 || leftJoy != 0.0 || rightJoy != 0.0) {
      robotElevator.setSpeed(triggers);
      robotWrist.setJointSpeed(rightJoy);
      robotShoulder.setJointSpeed(leftJoy);
      joystickCommand = true;
    }
    positionCommand = false;
  }
}
