package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPIDCommand extends Command {
    private Elevator elevatorSubsystem;
    private final PIDController pidController;

    public ElevatorPIDCommand(Elevator elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pidController = new PIDController(0.00001, 0.00, 0.00); // .04 .04 .0005
        pidController.setSetpoint(setpoint);
      }

  @Override
  public void initialize() {
    System.out.println("ElevatorPIDCommand started!");
    pidController.reset();
    elevatorSubsystem.setElevatorMotor(0);
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(elevatorSubsystem.getElevatorEncoder1());
    elevatorSubsystem.setElevatorMotor(speed);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevatorMotor(0);
    System.out.println("ElevatorPIDCommand ended!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
