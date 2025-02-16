package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.manipulator.ElevatorInterface.ElevatorPosition;
import frc.robot.subsystems.manipulator.WristInterface.WristAngle;

public class Manipulator extends SubsystemBase {
  // Set to "Blank" versions to disable each component
  private final HandInterface robotHand = new HandBlank();
  private final WristInterface robotWrist = new WristBlank();
  private final ElevatorInterface robotElevator = new Elevator();
  private final double elevatorHeightTolerance = 0.03;

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
    robotWrist.updateWrist();
    robotElevator.move();
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

  public void emergencyStop() {
    robotElevator.setSpeed(0.0);
    robotWrist.setWristSpeed(0.0);
    robotHand.stopMotors();
  }

  public void runManipulator(double LJ, double RJ) {

    // if we recieve a joystick input stop both auto positioning
    if (LJ == 0.0 && RJ == 0.0 && positionCommand) {
      robotElevator.setPosition(elevatorPositions[levelIndex]);
      robotWrist.setWristAngle(wristAngles[levelIndex]);
    } else if (LJ != 0.0 || RJ != 0.0) {
      robotElevator.setSpeed(LJ);
      robotWrist.setWristSpeed(RJ);
      joystickCommand = true;
    }
    positionCommand = false;
  }
}
