package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Coral extends SubsystemBase {
  // TODO: add default values
  private static final LoggedTunableNumber IRThresholdCoral =
      new LoggedTunableNumber("Coral/Threshold");

  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();
  private final CoralIO io;
  double threshold;

  public Coral(CoralIO io) {
    this.io = io;
  }

  private int integ = 0;

  public boolean hasCoral() {
    if (inputs.data.IrSensorValue() > threshold) {
      integ++;
    } else {
      integ = 0;
    }
    return integ > 5;
  }

  public Command run(double percentOut, boolean passiveActive) {
    return runEnd(
        () -> {
          io.setOutput(percentOut);
        },
        () -> {
          if (passiveActive) {
            io.setOutput(-.25);
          } else {
            io.setOutput(0);
          }
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Coral", inputs);

    // Update tunable numbers (configure before builds and during tuning)
    if (IRThresholdCoral.hasChanged(hashCode())) {
      threshold = IRThresholdCoral.get();
    }
  }
}
