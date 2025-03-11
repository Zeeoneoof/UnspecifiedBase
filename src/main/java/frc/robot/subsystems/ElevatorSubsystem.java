package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax m_elevatorMotorLeft = new SparkMax(ElevatorConstants.kElevatorMotorPortLeft, MotorType.kBrushless);
    private final SparkMax m_elevatorMotorRight = new SparkMax(ElevatorConstants.kElevatorMotorPortRight, MotorType.kBrushless);
    private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();
    private RelativeEncoder elevatorEncoder;
    private SparkClosedLoopController elevatorPIDController;

    public final DigitalInput rightLimitSwitch = new DigitalInput(0); // TODO: Wire and correct port
 
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

        elevatorConfig.closedLoop.maxMotion
            .maxVelocity(2)
            .maxAcceleration(1)
            .allowedClosedLoopError(40);
        
        followerConfig
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        followerConfig.follow(ElevatorConstants.kElevatorMotorPortLeft,true);
            


        m_elevatorMotorLeft.configure(elevatorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_elevatorMotorRight.configure(followerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        elevatorEncoder = m_elevatorMotorLeft.getEncoder();
        elevatorPIDController = m_elevatorMotorLeft.getClosedLoopController();


            
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Encoder Value", elevatorEncoder.getPosition());
    }

    public void runWithPosition(double position){
        elevatorPIDController.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl);
    }

    public void runWithVoltage(double voltage){
        m_elevatorMotorLeft.set(voltage);
    }

    public void zeroEncoder(){
        elevatorEncoder.setPosition(0);
    }

    public void stopMotor(){
        m_elevatorMotorLeft.stopMotor();
        m_elevatorMotorRight.stopMotor();
    }

    public double getMotorAmps(){
        return m_elevatorMotorLeft.getOutputCurrent();
    }

    

    
}
    
