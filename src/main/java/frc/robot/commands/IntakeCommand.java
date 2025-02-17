package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;

public class IntakeCommand extends Command {
    private Intake IntakeSubsystem;
    private LEDs ledSubsystem;
    
    public IntakeCommand(Intake IntakeSubsystem, LEDs ledSubsystem) {
        this.IntakeSubsystem = IntakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        IntakeSubsystem.setIntakeMotor(0);
        System.out.println("IntakeCommand started!");
    }

    @Override
    public void execute() {
        if (IntakeSubsystem.getPEStatus()) {
            IntakeSubsystem.setIntakeMotor(0);
            ledSubsystem.FlashLed();
        } else {
            IntakeSubsystem.setIntakeMotor(.4);
            ledSubsystem.SetLEDBuffer(255, 0, 0);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.setIntakeMotor(0);
        ledSubsystem.SetLEDBuffer(0, 0, 0);
        System.out.println("IntakeCommand ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
