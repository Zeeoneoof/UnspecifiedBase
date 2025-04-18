package frc.robot.subsystems;
import com.revrobotics.spark.ClosedLoopSlot;
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
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.LEDConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkFlex m_elevatorMotorLeft = new SparkFlex(ElevatorConstants.kElevatorMotorPortLeft, MotorType.kBrushless);
    private final SparkFlex m_elevatorMotorRight = new SparkFlex(ElevatorConstants.kElevatorMotorPortRight, MotorType.kBrushless);
    private final SparkMax m_climber = new SparkMax(10,MotorType.kBrushless);
    private final SparkFlexConfig elevatorConfig = new SparkFlexConfig();
    private final SparkFlexConfig followerConfig = new SparkFlexConfig();
    private final SparkMaxConfig climberConfig = new SparkMaxConfig();
    private RelativeEncoder elevatorEncoder;
    private SparkClosedLoopController elevatorPIDController;
    public final static CommandXboxController subXboxController = new CommandXboxController(1);
    private double universalOffset = 3;

    // Logical debouncer
    Debouncer m_debouncer = new Debouncer(3,DebounceType.kBoth);
    Debouncer m_debouncer2 = new Debouncer(3,DebounceType.kBoth);

    public ElevatorSubsystem(){
        // Configue climber motor
        climberConfig
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        // Configure the elevator motors
        elevatorConfig
            .voltageCompensation(12.0)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        elevatorConfig.closedLoop
            .p(0.3)
            .i(0.0)
            .d(0.014)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder); // Motor encoder

        elevatorConfig.closedLoop.maxMotion
        // Units are in Rotations Per Minute
            .maxVelocity(12*75*4)
            .maxAcceleration(6*240*1.9)
            .allowedClosedLoopError(0.2); // Rotations
        
        followerConfig
            .voltageCompensation(12)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        followerConfig.follow(ElevatorConstants.kElevatorMotorPortLeft,true);
            

        // Apply configs
        m_elevatorMotorLeft.configure(elevatorConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        m_elevatorMotorRight.configure(followerConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

        // Initialize control system variables
        elevatorEncoder = m_elevatorMotorLeft.getEncoder();
        elevatorPIDController = m_elevatorMotorLeft.getClosedLoopController();
    }

    @Override
    public void periodic(){
        // Random debug stuff
        SmartDashboard.putNumber("Encoder Value", elevatorEncoder.getPosition());
        SmartDashboard.putNumber("Motor Amps Left",m_elevatorMotorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Motor Amps Right", m_elevatorMotorRight.getOutputCurrent());
        SmartDashboard.putString("Game Mode (E):", "");
        
        
        // Stall detection (stop after stalled >3 seconds)
        if(m_debouncer.calculate((m_elevatorMotorLeft.getOutputCurrent()>38)||m_debouncer2.calculate(m_elevatorMotorRight.getOutputCurrent()>38))){
            stopMotor();
        }
        
    }

    public void runWithPosition(double position){

        elevatorPIDController.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,0);
    }


    public void runWithVoltage(double voltage){
        m_elevatorMotorLeft.set(voltage);
    }

    public void zeroEncoder(){
        elevatorEncoder.setPosition(0);
        stopMotor();
    }

    public void stopMotor(){
        m_elevatorMotorLeft.stopMotor();
        m_elevatorMotorRight.stopMotor();
    }

    public void spinClimbMotor(double speed){
        m_climber.set(speed);
    }
    public void stopClimbMotor(){
        m_climber.stopMotor();
    }

    public void manualAdjustmentFunc(double adjustment){
        double position = elevatorEncoder.getPosition();
        position += adjustment;
        double finalPosition = position;
        SmartDashboard.putNumber("Manually adjusted position", finalPosition);
        runWithPosition(finalPosition);
    }

    public double absDistanceFrom(double a, double b){
        return Math.abs(a-b);
    }

    public void setLEDProgress(int mode, double position){
        LEDPattern baseColorAlgae = LEDPattern.solid(Color.kPaleTurquoise);
        LEDPattern baseColorCoral = LEDPattern.solid(Color.kWhite);
        double distanceFrom = absDistanceFrom(position, elevatorEncoder.getPosition());
        LEDPattern progressBar = LEDPattern.progressMaskLayer(()->MathUtil.clamp(absDistanceFrom(position, elevatorEncoder.getPosition())/distanceFrom,0,1));
        if (mode == 1){
            LEDPattern finalPattern = baseColorCoral.mask(progressBar);
            finalPattern.applyTo(LEDConstants.m_ledBuffer);
            LEDConstants.m_led.setData(LEDConstants.m_ledBuffer);

        } else {
            LEDPattern finalPattern = baseColorAlgae.mask(progressBar);
            finalPattern.applyTo(LEDConstants.m_ledBuffer);
            LEDConstants.m_led.setData(LEDConstants.m_ledBuffer);
        }

    }



    /*
     * The elevator position controls
     */

    public Command setElevator(int mode, int controlType){
        // Coral
        if (controlType==1){
        switch(mode){
            case 0:
            return this.runOnce(()->runWithPosition(19.1))
                .alongWith(this.runOnce(()->setLEDProgress(1,19.1)));
            case 1:
            return this.runOnce(()->runWithPosition(37.38))
                .alongWith(this.runOnce(()->setLEDProgress(1,37.38)));
            case 2:
            return this.runOnce(()->runWithPosition(59.39))
                .alongWith(this.runOnce(()->setLEDProgress(1,59.39)));
            case 3:
            return this.runOnce(()->runWithPosition(85.54))
                .alongWith(this.runOnce(()->setLEDProgress(1,85.54)));
            case 4:
            return this.runOnce(()->runWithPosition(27.2))
                .alongWith(this.runOnce(()->setLEDProgress(1,27.2)));
            
        }
    }   else if (controlType==2){ // Algae Mode
        switch(mode){
            case 0:
            return this.runOnce(()->runWithPosition(18.85+3))
                .alongWith(this.runOnce(()->setLEDProgress(2,18.85+3)));
            case 1:
            return this.runOnce(()->runWithPosition(46.26+3))
                .alongWith(this.runOnce(()->setLEDProgress(2,46.26+3)));
            case 2:
            return this.runOnce(()->runWithPosition(61.07+3))
                .alongWith(this.runOnce(()->setLEDProgress(2,61.07+3)));
            case 3:
            return this.runOnce(()->runWithPosition(ElevatorConstants.bargeElevatorPosition+6))
                .alongWith(this.runOnce(()->setLEDProgress(2,ElevatorConstants.bargeElevatorPosition+6)));
            case 4:
            return this.runOnce(()->runWithPosition(6.06))
                .alongWith(this.runOnce(()->setLEDProgress(2,6.06)));
            
        }
    }
        return this.runOnce(()->stopMotor());
        
    }

    public Command resetEncoder(){
        return this.runOnce(()-> zeroEncoder());
    }

    public Command manualAdjustment(double adjustment){
        double position = elevatorEncoder.getPosition();
        position += adjustment;
        if (position>85.06){
            return Commands.none();
        }
        double finalPosition = position;
        SmartDashboard.putNumber("Manually adjusted position", finalPosition);
        return this.runOnce(()->runWithPosition(finalPosition));
    }

    



    

    
}
    
