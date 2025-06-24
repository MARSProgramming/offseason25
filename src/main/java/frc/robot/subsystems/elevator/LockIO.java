package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface LockIO {
  @AutoLog
  class LockIOInputs {
    public LockIOData data =
        new LockIOData(
            false,
            0.0,
            0.0);
  }

  record LockIOData(
      boolean limit,
      double servoAngle,
      double servoPosition
      ) {}

  default void updateInputs(LockIOInputs inputs) {}

  default void setServoAngle(double angleDeg) {}

  default void setServo(double position) {}

}
