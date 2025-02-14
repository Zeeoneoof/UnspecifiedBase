package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_elevatorMotor = new SparkMax(Constants.Elevator.kElevatorMotorPort, MotorType.kBrushless);
    private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    
    // Config Init
    public ElevatorSubsystem(){
        elevatorConfig
            .voltageCompensation(12.0)
            .
            
    }

    
}
