package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClawConstants;

public class PivotSubsystem extends SubsystemBase {
    private final SparkMax m_clawRotationMotor = new SparkMax(ClawConstants.kClawRotationPort, MotorType.kBrushless);
    private final SparkMaxConfig clawRotationConfig = new SparkMaxConfig();
    private AbsoluteEncoder clawRotationEncoder;
    private SparkClosedLoopController clawRotationPIDController;
    private final ArmFeedforward rotationFF = new ArmFeedforward(0.05, 1.8,0.06);

    public PivotSubsystem(){
        clawRotationConfig
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
            
        
        clawRotationConfig.closedLoop
            .pidf(0.38,0,0.122,0.1)
            .outputRange(-1, 1)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        
        clawRotationConfig.closedLoop.maxMotion
            .allowedClosedLoopError(0.005)
            .maxAcceleration(40*8)
            .maxVelocity(20*32);
        
        clawRotationConfig.softLimit
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false)
            .forwardSoftLimit(ClawConstants.clawRotationForwardLimit)
            .reverseSoftLimit(ClawConstants.clawRotationReverseLimit+0.02);
        
        m_clawRotationMotor.configure(clawRotationConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        clawRotationEncoder = m_clawRotationMotor.getAbsoluteEncoder();
        clawRotationPIDController = m_clawRotationMotor.getClosedLoopController();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Claw Rotation Amps", m_clawRotationMotor.getOutputCurrent());
        SmartDashboard.putString("Game Mode (C):", "");
        SmartDashboard.putNumber("Rotation Encoder", clawRotationEncoder.getPosition());

        SmartDashboard.putNumber("FF", rotationFF.calculate((clawRotationEncoder.getPosition()-.269)*2*Math.PI, 0));
    }   

    /**
     * Commands for claw rotation
     * @param mode 0: Low, 1: Mid, 2: High, 3: Special
     * @implNote controlType - 1: Coral Mode, 2: Barge Mode
     */
    public Command setPivot(int mode, int controlType){
        // Coral
        if (controlType==1){
            switch(mode){
                case 0: // Low
                return this.runOnce(()->runRotationWithPosition(0.26));
                case 1: // Mid
                return this.runOnce(()->runRotationWithPosition(0.2));
                case 2: // High
                return this.runOnce(()->runRotationWithPosition(0.175));
                case 3: // Special - Trough
                return this.runOnce(()->runRotationWithPosition(0.225));
                case 4:
                return this.runOnce(()->runRotationWithPosition(0.358));
            }
        } else if (controlType==2){ // Algae
            switch(mode){
                case 0: // Processor (Low)
                return this.runOnce(()->runRotationWithPosition(0.218));
                case 1: // Mid
                return this.runOnce(()->runRotationWithPosition(0.135));
                case 2: // High
                return this.runOnce(()->runRotationWithPosition(0.186));
                case 3: // Special - Barge
                return this.runOnce(()->runRotationWithPosition(0.461));
                case 4:
                return this.runOnce(()->runRotationWithPosition(0.22));
            }
        }
        
        return this.runOnce(()->stopRotationMotor());
        
    }

    /**
     * Sets the PID loop to reference a setpoint (the Absolute Encoder in this case)
     * @param position PID setpoint
     * 
     */
    public void runRotationWithPosition(double position){
       clawRotationPIDController.setReference(position, ControlType.kMAXMotionPositionControl);
    }

    /**
     * Stops claw yaw-axis rotation activity
     */
    public void stopRotationMotor(){
        m_clawRotationMotor.stopMotor();
    }

    /**
     * Method for checking claw yaw-axis motor amperage
     * @return Yaw-axis motor amperage
     */
    public double getRotationMotorAmps(){
        return m_clawRotationMotor.getOutputCurrent();
    }

    public void manualAdjustmentFunc(double adjustment){
        double position = clawRotationEncoder.getPosition();
        position += adjustment;
        double finalPosition = position;
        SmartDashboard.putNumber("Manually adjusted position (P)", finalPosition);
        clawRotationPIDController.setReference(finalPosition, ControlType.kMAXMotionPositionControl);
    }
    
    /**
     * 
     * @return Command chain to stow claw in upwards position
     */
    public Command stowCommand(){
        BooleanSupplier stowed = ()->clawRotationEncoder.getPosition()>=0.43;
        return this.runOnce(()->runRotationWithPosition(ClawConstants.clawStowedPosition)).andThen(Commands.waitUntil(stowed)).andThen(this.runOnce(()->stopRotationMotor()));
    }

    /**
     * 
     * @param adjustment The amount to change the setpoint by (see Absolute Encoder values)
     * @return Command to manually lower or raise the claw
     */
    public Command manualAdjustment(double adjustment){
        double position = clawRotationEncoder.getPosition();
        position += adjustment;
        final double finalPosition = position;
        return this.runOnce(()->runRotationWithPosition(finalPosition));
    }

}
