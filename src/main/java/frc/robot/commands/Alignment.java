package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.Color;
import frc.robot.subsystems.led.LED.LEDSection;
import frc.robot.util.Helpers;

public class Alignment extends Command {
  Drive drive;
  LED led;
  Pose2d pidTarget;
  boolean left;
  int level;
  Debouncer finishDebouncer;

  public Alignment(Drive mDrive, boolean mLeft, int mLevel) {
    drive = mDrive;
    left = mLeft;
    level = mLevel;

    finishDebouncer = new Debouncer(0.06);
  }

  @Override
  public void initialize() {
    pidTarget = Helpers.getNearestTarget(drive.getPose(), level, left);
  }

  @Override
  public void execute() {
    ChassisSpeeds currentSpeeds = drive.calculateRobotRelative(pidTarget);
    drive.runVelocity(currentSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    drive.stopWithX();
    led.setStrobe(new Color(255, 0, 0), LEDSection.CROSSBAR, 1);
    Timer.delay(1.5);
    led.clearSection(LEDSection.CROSSBAR);
  }

  @Override
  public boolean isFinished() {
    return finishDebouncer.calculate(
        drive.getPose().getTranslation().getDistance(pidTarget.getTranslation())
            < 0.06); // less than 6cm from target (tune as we go)
  }
}
