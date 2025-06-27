package frc.robot.subsystems.algae;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;

public class AlgaeIOSparkMax implements AlgaeIO {
  // TODO: add connection check
  private final SparkBase algaeMotor;
  private final SparkBaseConfig config;

  private final int currentLimit = 40;

  public AlgaeIOSparkMax() {
    algaeMotor = new SparkMax(Constants.RobotMap.SPARK.ALGAE, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.smartCurrentLimit(currentLimit);
    config.signals.busVoltagePeriodMs(20).outputCurrentPeriodMs(20).appliedOutputPeriodMs(20);
    tryUntilOk(
        algaeMotor,
        5,
        () ->
            algaeMotor.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(AlgaeIOInputs inputs) {
    inputs.data =
        new AlgaeIOData(
            algaeMotor.getAppliedOutput(),
            algaeMotor.getOutputCurrent(),
            algaeMotor.getBusVoltage(),
            algaeMotor.getMotorTemperature());
  }

  @Override
  public void setOutput(double percent) {
    algaeMotor.set(percent);
  }
}
