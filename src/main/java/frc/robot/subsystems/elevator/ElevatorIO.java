package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
 @AutoLog
 class ElevatorIOInputs {
   public ElevatorIOData data = new ElevatorIOData(
    false,
    0.0,
    0.0, 
    0.0, 
    0.0, 
    0.0, 
    0.0
    );
 }
    record ElevatorIOData(
        boolean motorConnected,
        double positionRad,
        double velocityRadPerSec,
        double appliedVolts,
        double torqueCurrentAmps,
        double supplyCurrentAmps,
        double tempCelsius
    ) {}
    
    default void updateInputs(ElevatorIOInputs inputs) {}

    default void runOpenLoop(double output) {}
  
    default void runVolts(double volts) {}
  
    default void stop() {}
  
    default void runPosition(double positionRad, double feedforward) {}
  
    default void setPID(double kP, double kI, double kD) {}
  
    default void setBrakeMode(boolean enabled) {}
} 