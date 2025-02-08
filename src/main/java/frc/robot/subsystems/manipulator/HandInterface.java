package frc.robot.subsystems.manipulator;

public interface HandInterface {

  public void intakeCoral();

  public void clockwise();

  public void counterClockwise();

  public void expelCoral();

  public void stopMotors();

  public boolean coralInPosition();

  public void intakeAlgae();

  public void expelAlgaeNet();

  public void expelAlgaeProcessor();

  public boolean algaeInPosition();
}
