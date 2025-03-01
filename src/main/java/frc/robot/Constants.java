// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
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

    // SPARK MAX CAN IDs 10-17
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
  }
  
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    //! remember to change between Neo drive and Votex drive
    //public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kNeoFreeSpeedRpm / 60;
    public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kVortexFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }


  public static final class CapstanConstants {
    public static final int kElevatorLeaderCanId = 18;
    public static final int kElevatorFollowerCanId = 19;
    public static final int kAlgaeWristCanId = 20;
    public static final int kCoralWristCanId = 21;

    public static final double kWristMotorReduction = 1.0;
    public static final double kElevatorMotorReduction = 1.0;

    public static final double kUpperLimit = 0.0;

    public static final class ElevatorSetpoints {
      // Elevator Positions
      public static final double kStore = 0;
      public static final double kFeederStation = 0;
      public static final double kProcessor = 0;
      public static final double kNet = 0;
      public static final double kL1 = 0;
      public static final double kL2 = 0;
      public static final double kL3 = 0;
      public static final double kL4 = 0;
    }

    public static final class AlgaeWristSetpoints {
      // Algae Wrist Positions
      public static final double kStore = 0;
      public static final double kFeederStation = 0;
      public static final double kProcessor = 0;
      public static final double kNet = 0;
      public static final double kL1 = 0;
      public static final double kL2 = 0;
      public static final double kL3 = 0;
      public static final double kL4 = 0;
    }

    public static final class CoralWristSetpoints {
      //Coral Wrist Positions
      public static final double kStore = 0;
      public static final double kFeederStation = 0;
      public static final double kProcessor = 0;
      public static final double kNet = 0;
      public static final double kL1 = 0;
      public static final double kL2 = 0;
      public static final double kL3 = 0;
      public static final double kL4 = 0;
    }
  }

  public final class CoralConstants {
    //CAN-ID's
    public static final int kCoralLeader = 5;
    public static final int kCoralFollower = 6;
    //speeds
    public static final double kFeederStationSpeed = 1.0;
    public static final double kL1Speed = -0.25;
    public static final double kL2Speed = -0.5;
    public static final double kL3Speed = -0.75;
    public static final double kL4Speed = -1.0;
  }

  public final class AlgaeConstants {
    //CAN-ID's
    public static final int kAlgaeLeader = 7;
    public static final int kAlgaeFollower = 8;
    public static final int kAlgaeLaser = 9;
    //speeds
    public static final double kNetSpeed = -1.0;
    public static final double kProcessorSpeed = -0.5;
    public static final double kReefSpeed = 1.0;
  } 


  public static final class MotorConstants {
    //Kv Values
    public static final int kVortexKv = 565;
    //Free Speeds
    public static final int kNeoFreeSpeedRpm = 5676;
    public static final int kVortexFreeSpeedRpm = 6784;
    //Smart Current Limits
    public static final int kNeo550SetCurrent = 20;//amps
    public static final int kNeoSetCurrent = 50;//amps
    public static final int kVortexSetCurrent = 50;//amps
  }
  

  /*Usb port Constants for Laptop */
  public final class UsbPort {
    public static final double kDriveDeadband = 0.05;
    public static final double kBabyModeWeight = 0.85;

    public static final int kTestingControler = 1;
    public static final int kOperatorControler = 2; // Change this to 
    public static final int kDriveControler = 3;// Change this to kDriveControler
    // public static final int kFlightJoystick = 4;
  }

  /** Constants for the gamepad joysticks & buttons */ 
  public static final class GamePad {
    // Joysticks and their axes
    public final class LeftStick {
        public static final int kLeftRight = 0;
        public static final int kUpDown = 1;
    }
    public final class RightStick {
        public static final int kLeftRight = 2;
        public static final int kUpDown = 3;
    }

    public final class Button {
        public static final int kX = 1;
        public static final int kA = 2;
        public static final int kB = 3;
        public static final int kY = 4;
        public static final int kLB = 5;
        public static final int kRB = 6;
        public static final int kLT = 7;
        public static final int kRT = 8;
        public static final int kBack = 9; 
        public static final int kStart = 10;
        //Joystick click:
        public static final int kLeftStickB = 11;
        public static final int kRightStickB = 12;
    } 
  }
}
