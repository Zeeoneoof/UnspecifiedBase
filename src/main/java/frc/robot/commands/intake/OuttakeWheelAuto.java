package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClawSubsystem;


public class OuttakeWheelAuto extends Command {
  private ClawSubsystem m_intake;

  public OuttakeWheelAuto(ClawSubsystem m_intake) {
    this.m_intake = m_intake;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.runWheelWithVoltage(0.2);
  }

  @Override
  public void execute() {
  }
  @Override
  public void end(boolean interrupted) {
    m_intake.stopWheelMotor();
  }

  @Override
  public boolean isFinished() {
    new WaitCommand(1).schedule();
    return true;
  }
}
