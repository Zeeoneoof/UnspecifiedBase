/* package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorZero extends Command{
    private final ElevatorSubsystem m_ElevatorSubsystem;

    public ElevatorZero(ElevatorSubsystem subsystem){
        m_ElevatorSubsystem = subsystem;
        addRequirements(m_ElevatorSubsystem);
    }

    @Override 
    public void initialize(){
        m_ElevatorSubsystem.runWithVoltage(-1); // TODO: Figure out which direction is DOWN
    }

    @Override
    public boolean isFinished(){
        // Check if motor stalled or if reset switch hit
        return (m_ElevatorSubsystem.leftLimitSwitch.get() || m_ElevatorSubsystem.rightLimitSwitch.get()) || (m_ElevatorSubsystem.getMotorAmps() > 40);
    }

    @Override
    public void end(boolean interrupted){
        // Zeros encoder values when switch hit, only if command uninterrupted
        if(!interrupted){
            m_ElevatorSubsystem.stopMotor();
            m_ElevatorSubsystem.zeroEncoder();
        } else{
            m_ElevatorSubsystem.stopMotor();
        }
    }
}
*/