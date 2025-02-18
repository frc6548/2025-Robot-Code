package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final SparkMax FrontElevatorMotor = new SparkMax(14, MotorType.kBrushless); //14
    private final SparkMax BackElevatorMotor = new SparkMax(15, MotorType.kBrushless); //15
    public final RelativeEncoder ElevatorEncoder1 = FrontElevatorMotor.getEncoder();
    public final RelativeEncoder ElevatorEncoder2 = BackElevatorMotor.getEncoder();

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Velocity1", ElevatorEncoder1.getVelocity());
        SmartDashboard.putNumber("Elevator Velocity2", ElevatorEncoder2.getVelocity());
        SmartDashboard.putNumber("Elevator Encoder1", ElevatorEncoder1.getPosition());
        SmartDashboard.putNumber("Elevator Encoder2", ElevatorEncoder2.getPosition());
}
    public void setElevatorMotor(double speed) {
        FrontElevatorMotor.set(speed);
        BackElevatorMotor.set(speed);
}
    public double getElevatorEncoder1() {
        return ElevatorEncoder1.getPosition();
}
    public void resetElevatorEncoder1() {
        ElevatorEncoder1.setPosition(0);
}
}