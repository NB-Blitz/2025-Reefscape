package frc.robot.subsystems.manipulator;

public interface ElevatorInterface {
  // create an enum for preset elevator heights (ex. coral level 1, 2, 3, 4)
  public enum ElevatorPosition {
    coralL1(100),
    coralL2(200),
    coralL3(300),
    coralL4(400),
    algaeL1(250),
    algaeL2(350),
    coralIntake(250),
    algaeBarge(500),
    algaeProcessor(50),
    algaeIntake(25),
    bottom(0),
    top(500);

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
