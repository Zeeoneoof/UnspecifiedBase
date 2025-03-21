// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ElevatorConstants {
    // CAN IDs
    public static final int kElevatorMotorPortLeft = 13; 
    public static final int kElevatorMotorPortRight = 14;

    // Elevator encoder position for coral
    public static final double coralHighElevatorPosition = 77;
    public static final double coralMidElevatorPosition = 51.1;
    public static final double coralLowElevatorPosition = 34.7;
    public static final double troughElevatorPosition = 20;
    public static final double coralIntakeElevatorPosition = 19.5;

    // Elevator encoder positions for algae
    public static final double bargeElevatorPosition = 78;
    public static final double algaeMidPosition = 38;
    public static final double algaeHighPosition = 51.5;
    
  }

  public static final class ClawConstants {
    // CAN
    public static final int kClawWheelPort = 11;
    public static final int kClawRotationPort = 12;

    // Claw Absolute Encoder positions
    public static final double clawStowedPosition = 0.45;
    public static final double clawCoralDepositPosition = 0.182; 
    public static final double clawAlgaeDepositPosition = 0.3;
    public static final double clawIntakePosition = 0.385;

    // Claw limits (forward is down, reverse is up)
    public static final double clawRotationForwardLimit = 0.460;
    public static final double clawRotationReverseLimit = 0;
    
  }

  public static final class DriveConstants {
    public static final boolean kGyroReversed = false;
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 9;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 2;

    // OLD
    /* 
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 9;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
    */
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSubsystemDriverControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class AlignmentConstants{
    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0.45; // RY
    public static final double X_SETPOINT_REEF_ALIGNMENT = -.19; // TX
    public static final double Y_SETPOINT_REEF_ALIGNMENT_LEFT = -0.55; // TY
    public static final double Y_SETPOINT_REEF_ALIGNMENT_RIGHT = 0; // TY
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.02;
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;
    
  }
}
