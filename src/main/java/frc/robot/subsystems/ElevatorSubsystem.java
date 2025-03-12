package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfigAccessor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkFlex m_elevatorMotorLeft = new SparkFlex(ElevatorConstants.kElevatorMotorPortLeft, MotorType.kBrushless);
    private final SparkFlex m_elevatorMotorRight = new SparkFlex(ElevatorConstants.kElevatorMotorPortRight, MotorType.kBrushless);
    private final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    private final SparkFlexConfig followerConfig = new SparkFlexConfig();
    private RelativeEncoder elevatorEncoder;
    private SparkClosedLoopController elevatorPIDController;

    // Logical debouncer
    Debouncer m_debouncer = new Debouncer(3,DebounceType.kRising);

    public ElevatorSubsystem(){
        // Configure the elevator motor
        elevatorConfig
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        elevatorConfig.closedLoop
            .p(0.01)
            .i(0.0)
            .d(0.01)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        elevatorConfig.closedLoop.maxMotion
            .maxVelocity(10)
            .maxAcceleration(7.5)
            .allowedClosedLoopError(20);
        
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
        SmartDashboard.putNumber("Motor Amps Left",m_elevatorMotorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Motor Amps Right", m_elevatorMotorRight.getOutputCurrent());
        if(m_debouncer.calculate((m_elevatorMotorLeft.getOutputCurrent()>50)||(m_elevatorMotorRight.getOutputCurrent()>50))){
            stopMotor();
        }
        
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

    /*
     * The elevator position controls
     */

    public Command setElevator(int level){
        switch (level){
            // Low
            case 1:
            return this.runOnce(()-> runWithPosition(ElevatorConstants.lowElevatorPosition));

            // Mid
            case 2:
            return this.runOnce(()-> runWithPosition(ElevatorConstants.midElevatorPosition));

            // High
            case 3:
            return this.runOnce(()-> runWithPosition(ElevatorConstants.highElevatorPosition));
           
            // Stop motors
            case 4:
            return this.runOnce(()-> stopMotor());
        }
        return this.runOnce(()-> stopMotor());
        
    }

    public Command resetEncoder(){
        return this.runOnce(()-> zeroEncoder());
    }
    

    
}
    
