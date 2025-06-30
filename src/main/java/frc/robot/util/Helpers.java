package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.DynamicConstants;
import java.util.ArrayList;
import java.util.List;

public class Helpers {
  private static AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private static List<Pose2d> coralPoses = new ArrayList<>();
  private static int[] blue = Constants.REEF_MAP.FIDUCIALS.BLUE_CORAL_TAGS;
  private static int[] red = Constants.REEF_MAP.FIDUCIALS.RED_CORAL_TAGS;

  public static boolean withinTolerance(double value, double target, double tolerance) {
    return Math.abs(value - target) <= tolerance;
  }

  public static void loadTargets() {
    DriverStation.getAlliance()
        .ifPresent(
            alliance -> {
              if (alliance == Alliance.Red) {
                for (int i = 0; i < red.length; i++) {
                  coralPoses.add(layout.getTagPose(i).get().toPose2d());
                }
              } else if (alliance == Alliance.Blue) {
                for (int i = 0; i < blue.length; i++) {
                  coralPoses.add(layout.getTagPose(i).get().toPose2d());
                }
              }
            });
  }

  public static Pose2d getNearestTarget(Pose2d currentPose, int level, boolean left) {
    Pose2d target = currentPose.nearest(coralPoses);
    if (level == 4) {
      return target.transformBy(
          new Transform2d(
              (left)
                  ? (DynamicConstants.AlignTransforms.LeftXL4)
                  : (DynamicConstants.AlignTransforms.RightXL4),
              (left)
                  ? (DynamicConstants.AlignTransforms.LeftYL4)
                  : (DynamicConstants.AlignTransforms.RightYL4),
              new Rotation2d(
                  (left)
                      ? (Math.toRadians(DynamicConstants.AlignTransforms.LeftRot))
                      : (Math.toRadians(DynamicConstants.AlignTransforms.RightRot)))));
    }

    if (level == 3) {
      return target.transformBy(
          new Transform2d(
              (left)
                  ? (DynamicConstants.AlignTransforms.LeftXL3)
                  : (DynamicConstants.AlignTransforms.RightXL3),
              (left)
                  ? (DynamicConstants.AlignTransforms.LeftYL3)
                  : (DynamicConstants.AlignTransforms.RightYL3),
              new Rotation2d(
                  (left)
                      ? (Math.toRadians(DynamicConstants.AlignTransforms.LeftRot))
                      : (Math.toRadians(DynamicConstants.AlignTransforms.RightRot)))));
    }

    if (level == 2) {
      return target.transformBy(
          new Transform2d(
              (left)
                  ? (DynamicConstants.AlignTransforms.LeftXL2)
                  : (DynamicConstants.AlignTransforms.RightXL2),
              (left)
                  ? (DynamicConstants.AlignTransforms.LeftYL2)
                  : (DynamicConstants.AlignTransforms.RightYL2),
              new Rotation2d(
                  (left)
                      ? (Math.toRadians(DynamicConstants.AlignTransforms.LeftRot))
                      : (Math.toRadians(DynamicConstants.AlignTransforms.RightRot)))));
    }

    if (level == 1) {
      return target.transformBy(
          new Transform2d(
              (left)
                  ? (DynamicConstants.AlignTransforms.LeftXL1)
                  : (DynamicConstants.AlignTransforms.RightXL1),
              (left)
                  ? (DynamicConstants.AlignTransforms.LeftYL1)
                  : (DynamicConstants.AlignTransforms.RightYL1),
              new Rotation2d(
                  (left)
                      ? (Math.toRadians(DynamicConstants.AlignTransforms.LeftRot))
                      : (Math.toRadians(DynamicConstants.AlignTransforms.RightRot)))));
    } else {
      return currentPose;
    }
  }
}
