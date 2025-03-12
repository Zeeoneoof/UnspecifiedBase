 package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    // TODO: Figure out more complex claw rotation

    private final SparkMax m_clawWheelMotor = new SparkMax(ClawConstants.kClawWheelPort, MotorType.kBrushless);
    private final SparkMax m_clawRotationMotor = new SparkMax(ClawConstants.kClawRotationPort, MotorType.kBrushless);
    private final SparkMaxConfig clawWheelConfig = new SparkMaxConfig();
    private final SparkMaxConfig clawRotationConfig = new SparkMaxConfig();
    private AbsoluteEncoder clawRotatationEncoder;
    private SparkClosedLoopController clawRotationPIDController;


    public final DigitalInput clawLimitSwitch = new DigitalInput(0); // TODO: Wire and correct port
 
    public ClawSubsystem(){
        // Configure the claw motors
        clawWheelConfig
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        
        m_clawWheelMotor.configure(clawWheelConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        clawRotationConfig
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
            
        
        clawRotationConfig.closedLoop
            .p(0.1)
            .i(0.0)
            .d(0.01)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        
        clawRotationConfig.closedLoop.maxMotion
            .maxVelocity(10)
            .maxAcceleration(5)
            .allowedClosedLoopError(6);
        
        clawRotationConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(ClawConstants.clawRotationForwardLimit) // TODO: Find this value
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(ClawConstants.clawRotationReverseLimit); //TODO: Find this value
        
        
        m_clawRotationMotor.configure(clawRotationConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        clawRotatationEncoder = m_clawRotationMotor.getAbsoluteEncoder();
        clawRotationPIDController = m_clawRotationMotor.getClosedLoopController();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Claw Amps", m_clawWheelMotor.getOutputCurrent());
        SmartDashboard.putNumber("Claw Rotation Amps", m_clawRotationMotor.getOutputCurrent());
    }

    public void runWheelWithVoltage(double voltage){
        m_clawWheelMotor.set(voltage);
    }

    public void runRotationWithPosition(double position){
        clawRotationPIDController.setReference(position, ControlType.kPosition);
    }

    public void stopMotor(){
        m_clawWheelMotor.stopMotor();
    }

    public double getWheelMotorAmps(){
        return m_clawWheelMotor.getOutputCurrent();
    }

    public double getRotationMotorAmps(){
        return m_clawRotationMotor.getOutputCurrent();
    }

    

    
}
    
