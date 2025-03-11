/* package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.Constants.ClawConstants;

public class ClawOuttake extends Command{
    private final ClawSubsystem m_ClawSubsystem;
    WaitCommand wait = new WaitCommand(5);

    public ClawOuttake(ClawSubsystem subsystem){
        m_ClawSubsystem = subsystem;
        addRequirements(m_ClawSubsystem);
    }

    @Override 
    public void initialize(){
        m_ClawSubsystem.runWheelWithVoltage(-ClawConstants.clawWheelSpeed); // TODO: Figure out direction
        m_ClawSubsystem.runRotationWithPosition(ClawConstants.clawRotationHandoffPosition); // TODO: Figure out position
    }

    @Override
    public boolean isFinished(){
        wait.schedule();
        return true;
    }

    @Override
    public void end(boolean interrupted){
        m_ClawSubsystem.stopMotor();
    }
}
*/