package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.IntakeSystem;

public class ExtendIntake extends CommandBase {
  private IntakeSystem intake;

  public ExtendIntake(IntakeSystem intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intake.toggleCollapse();
  }

  @Override
  public void end(boolean interrupted) {
    intake.setExtendIntakeMotorVelocity(0);
    intake.switchIntakeState();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
