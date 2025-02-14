package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorLow extends Command{
    private final ElevatorSubsystem m_ElevatorSubsystem;
    WaitCommand wait = new WaitCommand(5);

    public ElevatorLow(ElevatorSubsystem subsystem){
        m_ElevatorSubsystem = subsystem;
        addRequirements(m_ElevatorSubsystem);
    }

    @Override 
    public void initialize(){
        m_ElevatorSubsystem.runWithPosition(ElevatorConstants.lowElevatorPosition); // TODO: Figure out low position
    }

    @Override
    public boolean isFinished(){
        wait.schedule();
        return true;
    }

    @Override
    public void end(boolean interrupted){
        m_ElevatorSubsystem.stopMotor();
    }
}