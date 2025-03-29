package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.manipulator.Manipulator;

public class ExpelCoral extends Command {
  private Manipulator manipulator;
  private Timer timer;

  public ExpelCoral(Manipulator manipulator) {
    this.manipulator = manipulator;
    timer = new Timer();
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    while (timer.get() < 3) {
      manipulator.expelCoralIntakeAlgae();
    }
  }

  @Override
  public boolean isFinished() {
    boolean isDone = timer.get() >= 3;
    if (isDone) {
      timer.stop();
      manipulator.stopHand();
    }
    return isDone;
  }
}
