package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final SparkMax IntakeMotor = new SparkMax(14, MotorType.kBrushless);
    private final SparkMax PivotMotor = new SparkMax(15, MotorType.kBrushless);
    public final RelativeEncoder IntakeEncoder = IntakeMotor.getEncoder();
    public final RelativeEncoder PivotEncoder = PivotMotor.getEncoder();
    public final DigitalInput Input = new DigitalInput(0);

     @Override
     public void periodic() {
        SmartDashboard.putNumber("Intake Velocity", IntakeEncoder.getVelocity());
        SmartDashboard.putNumber("Pivot Velocity", PivotEncoder.getVelocity());

        SmartDashboard.putNumber("Pivot Encoder", PivotEncoder.getPosition());

        SmartDashboard.putBoolean("Photoelectric Sensor Status", Input.get());
     }

     public void setIntakeMotor(double speed) {
      IntakeMotor.set(speed);
    }  

    public void setPivotMotor(double speed) {
      PivotMotor.set(speed);
    }  

    public double getPivotEncoder() {
      return PivotEncoder.getPosition();
    }

    public boolean getPEStatus(){
      return Input.get();
    }

    public void resetPivotEncoder() {
      PivotEncoder.setPosition(0);
    }  
}
