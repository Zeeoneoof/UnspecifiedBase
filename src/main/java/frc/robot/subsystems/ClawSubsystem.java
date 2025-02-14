package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    // TODO: Figure out more complex claw rotation

    private final SparkMax m_clawWheelMotor = new SparkMax(ClawConstants.kClawWheelPort, MotorType.kBrushless);
    private final SparkMaxConfig clawConfig = new SparkMaxConfig();

    public final DigitalInput clawLimitSwitch = new DigitalInput(0); // TODO: Wire and correct port
 
    public ClawSubsystem(){
        // Configure the claw motors
        clawConfig
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(45);


        m_clawWheelMotor.configure(clawConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
     
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Claw Amps", m_clawWheelMotor.getOutputCurrent());
    }

    public void runWithVoltage(double voltage){
        m_clawWheelMotor.set(voltage);
    }

    public void stopMotor(){
        m_clawWheelMotor.stopMotor();
    }

    public double getMotorAmps(){
        return m_clawWheelMotor.getOutputCurrent();
    }

    

    
}
