package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.MotorConstants;

public final class Configs {
    public static final class MAXSwerveModule {
      //public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
      public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
      public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

      static {
        // Use module constants to calculate conversion factors and feed forward gain.
        double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                / ModuleConstants.kDrivingMotorReduction;
        double turningFactor = 2 * Math.PI;
        double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

        drivingConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
        drivingConfig.encoder
                .positionConversionFactor(drivingFactor) // meters
                .velocityConversionFactor(drivingFactor / 60.0); // meters per second
        drivingConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.04, 0, 0)
                .velocityFF(drivingVelocityFeedForward)
                .outputRange(-1, 1);

        turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);
        turningConfig.absoluteEncoder
                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of the steering motor in the MAXSwerve Module.
                .inverted(true)
                .positionConversionFactor(turningFactor) // radians
                .velocityConversionFactor(turningFactor / 60.0); // radians per second
        turningConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1, 0, 0)
                .outputRange(-1, 1)
                // Enable PID wrap around for the turning motor. This will allow the PID
                // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                // to 10 degrees will go through 0 rather than the other direction which is a
                // longer route.
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);
      }
    }

    public static final class AlgaeConfig {
      public static final SparkMaxConfig algaeConfig = new SparkMaxConfig();
      static {
        algaeConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(MotorConstants.kNeo550SetCurrent);
        algaeConfig.encoder
          .positionConversionFactor(0) //what gear ratio, radians
          .velocityConversionFactor(0); // radians per second, radians/60
      }

    }

    public static final class CoralConfig {
      public static final SparkMaxConfig coralConfig = new SparkMaxConfig();
      
      static {
        coralConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(MotorConstants.kNeo550SetCurrent);
        coralConfig.encoder
          .positionConversionFactor(0) //what gear ratio, radians
          .velocityConversionFactor(0); // radians per second, radians/60
      }
    }
}
