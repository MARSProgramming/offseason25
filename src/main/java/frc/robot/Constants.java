// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final boolean tuningMode = false;
  public static final boolean disableHAL = false;

  public static class ElevatorConfigs {
    public static final double INCH_PER_ROT = 3;
    public static final double MAX_LIMIT = 8.1;
    public static final double REVERSE_LIMIT = 0;
    public static final double ROTATION_DEADZONE_ALGAE = 0.1;
    public static final double ROTATION_DEADZONE_CORAL = 0.05;
  }

  public static class PathPlanner {
    public static final double holonomicXkP = 5;
    public static final double holonomicXkI = 0;
    public static final double holonomicXkD = 0;
    public static final double holonomicYkP = 5;
    public static final double holonomicYkI = 0;
    public static final double holonomicYkD = 0;
    public static final double holonomicOkP = 3.0;
    public static final double holonomicOkI = 0.0;
    public static final double holonomicOkD = 0.0;
    public static final double holonomicOMaxVelocity = 5;
    public static final double holonomicOMaxAcceleration = 5;
  }

  public static class Alignment {
    public static final Distance kMinimumXYAlignDistance = Units.Meters.of(2);
    public static final Distance kPointTolerance = Units.Inches.of(0.5);
    public static final Distance kAlignmentTolerance = Units.Inches.of(1.5);
    public static final Angle kRotTolerance = Units.Degrees.of(2.5);
    public static final AngularVelocity kMaximumRotSpeed = Units.DegreesPerSecond.of(360);
    public static final LinearVelocity TeleoperatedMaximumVelocity =
        Units.MetersPerSecond.of(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.7);
    public static final double kMinimumElevatorMultiplier = 0.1;

    public static final PIDController kXController = new PIDController(3.5, 0, 0.0);
    public static final PIDController kYController = new PIDController(3.5, 0, 0.0);
    public static final ProfiledPIDController kRotController =
        new ProfiledPIDController(
            3.0,
            0,
            0.1,
            new TrapezoidProfile.Constraints(
                kMaximumRotSpeed.in(Units.DegreesPerSecond),
                Math.pow(kMaximumRotSpeed.in(Units.DegreesPerSecond), 2)));

    static {
      kXController.setTolerance(kPointTolerance.in(Units.Meters));
      kYController.setTolerance(kPointTolerance.in(Units.Meters));
      kRotController.enableContinuousInput(0, 360);
      kRotController.setTolerance(kRotTolerance.in(Units.Degrees));
    }

    public static HolonomicDriveController AlignmentController =
        new HolonomicDriveController(kXController, kYController, kRotController);
  }

  public static class RobotMap {

    public class DIO {
      public static final int LIMIT = 0;
    }

    public class PWM {
      public static final int SERVO = 4;
    }

    public class CAN {
      public static final CANBus SECONDARY_BUS = new CANBus("CAN-2", "./logs/example.hoot");

      public static final int CORAL = 12;
      public static final int ELEVATOR_MASTER = 18;
      public static final int ELEVATOR_FOLLOWER = 19;

      public static final int PIGEON_LEFT = 20;
      public static final int CANDLE = 40;

      public static final int FRONT_LEFT_DRIVE = 2;
      public static final int FRONT_LEFT_STEER = 3;
      public static final int BACK_LEFT_DRIVE = 4;
      public static final int BACK_LEFT_STEER = 5;
      public static final int FRONT_RIGHT_DRIVE = 8;
      public static final int FRONT_RIGHT_STEER = 9;
      public static final int BACK_RIGHT_DRIVE = 6;
      public static final int BACK_RIGHT_STEER = 7;
    }

    public class SPARK {
      public static final int ALGAE = 11;
    }

    public class VISION {
      public static final String reefCameraName = "reef_cam";
      public static final String feederCameraName = "feeder_cam";

      public static final Transform3d feederRobotToCam =
          new Transform3d(
              Units.Inches.of(-6),
              Units.Inches.of(9.06),
              Units.Inches.of(11.55),
              new Rotation3d(Units.Degrees.of(0), Units.Degrees.of(0), Units.Degrees.of(125)));

      public static final Transform3d reefRobotToCam =
          new Transform3d(
              Units.Inches.of(-6.3),
              Units.Inches.of(9.49),
              Units.Inches.of(13.44),
              new Rotation3d(Units.Degrees.of(-10), Units.Degrees.of(0), Units.Degrees.of(90)));
    }
  }

  public static class REEF_MAP {
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public class FIDUCIALS {
      public static final int RED_RIGHT_FEEDER_TAG = 1;
      public static final int RED_LEFT_FEEDER_TAG = 2;
      public static final int BLUE_RIGHT_FEEDER_TAG = 13;
      public static final int BLUE_LEFT_FEEDER_TAG = 12;

      public static final int RED_PROCESSOR_TAG = 3;
      public static final int BLUE_PROCESSOR_TAG = 16;

      public static final int BLUE_BACK_CLIMBING_TAG = 4;
      public static final int RED_FRONT_CLIMBING_TAG = 5;
      public static final int BLUE_FRONT_CLIMBING_TAG = 14;
      public static final int RED_BACK_CLIMBING_TAG = 15;

      public static final int RED_FL_CORAL_TAG = 6;
      public static final int RED_F_CORAL_TAG = 7;
      public static final int RED_FR_CORAL_TAG = 8;
      public static final int RED_BL_CORAL_TAG = 9;
      public static final int RED_B_CORAL_TAG = 10;
      public static final int RED_BR_CORAL_TAG = 11;

      public static final int BLUE_FR_CORAL_TAG = 17;
      public static final int BLUE_F_CORAL_TAG = 18;
      public static final int BLUE_FL_CORAL_TAG = 19;
      public static final int BLUE_BL_CORAL_TAG = 20;
      public static final int BLUE_B_CORAL_TAG = 21;
      public static final int BLUE_BR_CORAL_TAG = 22;

      public static final int[] RED_CORAL_TAGS = {6, 7, 8, 9, 10, 11};
      public static final int[] BLUE_CORAL_TAGS = {17, 18, 19, 20, 21, 22};
      public static final int[] RED_FEEDER_TAGS = {1, 2};
      public static final int[] BLUE_FEEDER_TAGS = {12, 13};
      public static final int[] RED_SIDE_CLIMB_TAGS = {4, 5};
      public static final int[] BLUE_SIDE_CLIMB_TAGS = {14, 15};
      public static final int[] PROCESSOR_TAGS = {1, 15};
    }
  }
}
