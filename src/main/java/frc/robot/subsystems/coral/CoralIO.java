package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

public interface CoralIO {
  @AutoLog
  class CoralIOInputs {
    public CoralIOData data = new CoralIOData(0.0, 0.0, 0.0);
  }

  record CoralIOData(double motorOutputPercent, double IrSensorValue, double motorStatorCurrent) {}

  default void updateInputs(CoralIOInputs inputs) {}

  default void setOutput(double percent) {}
  
}
