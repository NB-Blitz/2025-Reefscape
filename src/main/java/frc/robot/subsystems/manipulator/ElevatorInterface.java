package frc.robot.subsystems.manipulator;

public interface ElevatorInterface {
  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum ElevatorPosition {
    coralL1(0.2),
    coralL2(0.4),
    coralL3(0.6),
    coralL4(0.8),
    algaeL1(0.5),
    algaeL2(0.7),
    coralIntake(0.5),
    algaeBarge(1),
    algaeProcessor(0.15),
    algaeIntake(0.1),
    bottom(0),
    top(1);

    public final double position;

    ElevatorPosition(double position) {
      this.position = position;
    }
  }

  // returns the state of the limit switch
  public boolean getLimit();

  // moves the elevator to a preset position specified by the Position parameter (created in the
  // enum)
  public void setPosition(ElevatorPosition position);

  public void setSpeed(double joystickInput);

  // moves the elevator a certain speed according to the double parameter
  public void move();

  public double getHeight();
}
