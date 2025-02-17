package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakePivotPIDCommand extends Command {
    private Intake IntakeSubsystem;
    private final PIDController pidController;

    public IntakePivotPIDCommand(Intake IntakeSubsystem, double setpoint) {
        this.IntakeSubsystem = IntakeSubsystem;
        this.pidController = new PIDController(0.045, 0.014, 0.002); // .04 .04 .0005
        pidController.setSetpoint(setpoint);
      }

  @Override
  public void initialize() {
    System.out.println("IntakePivotCommand started!");
    pidController.reset();
    IntakeSubsystem.setPivotMotor(0);
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(IntakeSubsystem.getPivotEncoder());
    IntakeSubsystem.setPivotMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.setIntakeMotor(0);
    System.out.println("IntakePivotCommand ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
