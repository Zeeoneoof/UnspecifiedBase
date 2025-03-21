package frc.robot.subsystems;
import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;

public class ClawSubsystem extends SubsystemBase {
    // TODO: Figure out more complex claw rotation

    private final SparkMax m_clawWheelMotor = new SparkMax(ClawConstants.kClawWheelPort, MotorType.kBrushless);
    
    private final SparkMaxConfig clawWheelConfig = new SparkMaxConfig();
    
    


 
    public ClawSubsystem(){
        // Configure the claw motors
        clawWheelConfig
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);
        
        m_clawWheelMotor.configure(clawWheelConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Claw Amps", m_clawWheelMotor.getOutputCurrent());

        if(m_clawWheelMotor.getOutputCurrent()>45){
            stopWheelMotor();
        }
    }
    /**
     * A basic method to run the wheels on the claw
     * @param voltage Domain: [-1,1] - Percentage based speed
     */
    public void runWheelWithVoltage(double voltage){
        m_clawWheelMotor.set(voltage);
    }

    
    /**
     * Stops claw intake rotation activity
     */
    public void stopWheelMotor(){
        m_clawWheelMotor.stopMotor();
    }

    /**
     * Method for checking claw intake motor amperage
     * @return Intake motor amperage
     */
    public double getWheelMotorAmps(){
        return m_clawWheelMotor.getOutputCurrent();
    }


    
}
    
    
