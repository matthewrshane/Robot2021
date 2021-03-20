// Copyright (c) Team 564.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/** Constants for ports on the robot and driver station, and for each subsystem. */
public final class Constants {
  /**
   * Constants for ports that are on the roboRIO.
   *
   * <p>The SPI port to which the Gyroscope is connected is not included here because that constant
   * is provided by WPILib.
   */
  public static final class RoboRIO {
    /** Constants for the Control Area Network/CAN bus. */
    public static final class CAN {
      /** The port of the front left drive motor. */
      public static final int kPortMotorDriveFrontLeft = 1;
      /** The port of the back left drive motor. */
      public static final int kPortMotorDriveBackLeft = 2;
      /** The port of the front right drive motor. */
      public static final int kPortMotorDriveFrontRight = 3;
      /** The port of the back right drive motor. */
      public static final int kPortMotorDriveBackRight = 4;
      /** The ports of the launcher cannon double solenoid. */
      public static final int[] kPortsDoubleSolenoidLauncherCannon = {5, 6};
      /** The port of the intake motor. */
      public static final int kPortMotorIntake = 7;
      /** The port of the left launcher motor. */
      public static final int kPortMotorLauncherLeft = 8;
      /** The port of the conveyor belt. */
      public static final int kPortMotorBelt = 10;
      /**
       * The port of the right launcher motor. On the electronics board, this is #8b, but this
       * couldn't be replicated due to software limitaitons.
       */
      public static final int kPortMotorLauncherRight = 11;
    }

    /** Constants for the Digital Input/Output ports. */
    public static final class DIO {
      /** The ports of the left drive encoder. */
      public static final int[] kPortsEncoderDriveLeft = {0, 1};
      /** The ports of the right drive encoder. */
      public static final int[] kPortsEncoderDriveRight = {2, 3};
      /** The port of the photoelectric sensor at the bottom of the storage, where balls enter. */
      public static final int kPortPhotoelectricSensorEnter = 4;
      /** The port of the photoelectric sensor at the top of the storage, where balls leave. */
      public static final int kPortPhotoelectricSensorExit = 5;
    }

    /** Constants for the Analog Input ports. */
    public static final class AI {
      /** The port of the ultrasonic sensor. */
      public static final int kPortUltrasonicSensorPort = 0;
    }
  }

  /** Constants for ports that are on the driver station. */
  public static final class DriverStation {
    /** The port of the driver controller. */
    public static final int kPortControllerDrive = 0;
    /** The port of the manipulator controller. */
    public static final int kPortControllerManip = 1;
  }

  /**
   * Constants for the drivetrain subsystem.
   *
   * <p>These values are susceptible to unit errors, so unit comments are required for all of these.
   */
  public static final class DrivetrainConstants {
    // Speed Constants

    /**
     * The slow speed to drive at.
     *
     * <p>Unitless.
     */
    public static final double kSpeedSlow = 0.5;
    /**
     * The normal speed to drive at.
     *
     * <p>Unitless.
     */
    public static final double kSpeedNormal = 0.85;
    /**
     * The fast speed to drive at.
     *
     * <p>Unitless.
     */
    public static final double kSpeedFast = 0.95;
    /**
     * The max velocity to drive at while path following.
     *
     * <p>Unit: Meters / second.
     */
    public static final double kMaxTrajectoryVelocity = 3;
    /**
     * The max acceleration to drive with while path following.
     *
     * <p>Unit: Meters / Second^2.
     */
    public static final double kMaxTrajectoryAcceleration = 3;
    /**
     * The maximum voltage available to the drive motors while following a path.
     *
     * <p>Unit: Volts.
     */
    public static final double kMaxVolts = 10;

    // Physical Robot Constants

    /**
     * The radius of wheels.
     *
     * <p>Unit: Meters.
     */
    public static final double kWheelRadius = Units.inchesToMeters(6);
    /**
     * The trackwidth of the drivetrain.
     *
     * <p>Unit: Meters.
     */
    public static final double kTrackWidth = 1.189654643399417;
    /**
     * The gear ratio between the shaft of the motor, and the encoder and wheels.
     *
     * <p>Unitless.
     */
    public static final double kGearRatio = 8.45;

    // Encoder Constants

    /**
     * The cycles per revolution outputted by the encoder.
     *
     * <p>Unit: Cycles / Revolution, or Pulses / Revolutions.
     */
    private static final int kEncoderCyclesPerRev = 2048;
    /**
     * The distance traveled in one revolution, equivalent to the circumference of the wheels.
     *
     * <p>Unit: Meters / Revolution.
     */
    private static final double kMetersPerRev = Math.PI * kWheelRadius;
    /**
     * The distance traveled in one pulse.
     *
     * <p>This constant does not depend on the gear ratio because the encoder is placed after the
     * gearbox.
     *
     * <p>Unit: Meters / Pulse, or Meters / Cycle. This will get multiplied by the pulse reading,
     * and then becomes just meters.
     */
    public static final double kEncoderDistancePerPulse = kMetersPerRev / kEncoderCyclesPerRev;
    /**
     * The number of encoder samples to average. This is greater than the default, to reduce noise.
     *
     * <p>Unit: Samples.
     */
    public static final int kEncoderSamplesToAverage = 5;

    /** Feedforward Constants */
    public static final class Feedforward {
      /**
       * The voltage required to overcome the motor's static friction.
       *
       * <p>Unit: Volts.
       */
      public static final double kS = 0.933;
      /**
       * The voltage required to cruise at a certain velocity. This is a function of the velocity
       * because the friction that has to be overcome increases as the robot speeds up.
       *
       * <p>Unit: Volts * (Seconds / Meter).
       */
      public static final double kV = 2.47;
      /**
       * The voltage required to induce a certain acceleration. This is a function of the
       * acceleration because, well, we need more volts to make the motor move faster ;)
       *
       * <p>Unit: Volts * (Seconds^2 / Meter).
       */
      public static final double kA = 0.548;
      /**
       * The voltage required to cruise at a certain angular velocity. This is a function of the
       * velocity because the friction that has to be overcome increases as the robot speeds up.
       *
       * <p>TODO: Determine this.
       *
       * <p>Unit: Volts * (Seconds / Radians).
       */
      public static final double kVAngular = 1.5;
      /**
       * The voltage required to induce a certain angular acceleration. This is a function of the
       * acceleration because, well, we need more volts to make the motor move faster ;)
       *
       * <p>TODO: Determine this.
       *
       * <p>Unit: Volts * (Seconds^2 / Radians).
       */
      public static final double kAAngular = 0.3;
    }

    /** Feedback Constants */
    public static final class Feedback {
      /**
       * The proportional term for the drive velocity PID controller.
       *
       * <p>Unit: Meters / Second.
       */
      public static final double kP = 2.55;
      /**
       * The B term for the Ramsete controller.
       *
       * <p>Unit: Meters.
       */
      public static final double kRamseteB = 2;
      /**
       * The ζ term for the Ramsete controller.
       *
       * <p>Unit: Seconds.
       */
      public static final double kRamseteZeta = 0.7;
    }
  }

  /** Constants for the intake subsystem. */
  public static final class IntakeConstants {
    /** The speed to intake balls at. */
    public static final double kSpeedIntake = 0.95;
    /** The speed to advance balls through the belt at. */
    public static final double kSpeedBelt = 1.0;
  }

  /** Constants for the shooter subsystem. */
  public static final class ShooterConstants {
    /** The slow speed to shoot balls at. */
    public static final double kSpeedSlow = 0.4;
    /** The normal speed to shoot balls at. */
    public static final double kSpeedNormal = 1.0;
  }

   /** Constants for the Vision subsystem. */
   public static final class VisionConstants {
    /** The height, in meters, of the camera from the field in the low shooting mode. Robot measurements used: 28.5in to screw holes + 1.75in to the camera = 30.25in */
    public static final double kHeightCameraLow = Units.inchesToMeters(30.25);
    /** The height, in meters, of the camera from the field in the high shooting mode. Robot measurements used: 34.25 in to screw holes + 1.75in to the camera = 36in */
    public static final double kHeightCameraHigh = Units.inchesToMeters(36);
    /** The height, in meters, of the top of the target, or the center of the goal, from the field. Field measurements used: 8ft 2 1/4in (98.25in) */
    public static final double kHeightTarget = Units.inchesToMeters(98.25);
    /** The pitch, in degrees, of the camera in the low shooting mode. Robot measurements used: 18 deg arm - 28deg camera = -10deg*/
    public static final double kPitchCameraLow = -10;
    /** The pitch, in degrees, of the camera in the high shooting mode. Robot measurements used: 31 deg arm - 28deg camera = 3deg*/
    public static final double kPitchCameraHigh = 3;
  }
}
