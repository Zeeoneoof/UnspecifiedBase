package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_elevatorMotor = new SparkMax(Constants.Elevator.kElevatorMotorPort, MotorType.kBrushless);
    private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    
    public ElevatorSubsystem(){
        // Configure the elevator motor
        elevatorConfig
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        elevatorConfig.closedLoop
        .p(0.1)
        .i(0.0)
        .d(0.01)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        m_elevatorMotor.configure(elevatorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

            
    }
    

    
}
