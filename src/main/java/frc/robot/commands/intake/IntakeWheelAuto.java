package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;


public class IntakeWheelAuto extends Command {
  private ClawSubsystem m_intake;
  public final DigitalInput intakeLimitSwitch = new DigitalInput(1);

  public IntakeWheelAuto(ClawSubsystem m_intake) {
    
    this.m_intake = m_intake;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.runWheelWithVoltage(-0.8);
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
    return intakeLimitSwitch.get();
  }
}
