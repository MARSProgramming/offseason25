package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.led.LED;

public class AutomationFactory {
    Elevator mElevator;
    Drive mDrive;
    Coral mCoral;
    Algae mAlgae;
    LED mLed;

    public Command alignAndRaiseElevator(int lev, boolean left) {
        return Commands.parallel(
            new Alignment(mDrive, true, lev),
            mElevator.setpointAndHold(lev)
        );
    }

    public Command AutoScore(int lev, boolean left, boolean passive) {
        Alignment thisAlign = new Alignment(mDrive, left, lev);
        return Commands.sequence(
            Commands.parallel(
            thisAlign,
            mElevator.setpointAndHold(lev).withTimeout(4)
        )
        ).andThen(
                mCoral.run(0.8, true).withTimeout(0.6)
        );
    }

    
}
