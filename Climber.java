package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class Climber {

    // two talons // control height
    // two spark maxes // control angle

    // WPI_TalonFX frontHeightMotor = new WPI_TalonFX(Constants.frontClimberHeightMotorIndex);
    //CANSparkMax frontAngleMotor = new CANSparkMax(Constants.frontClimberAngleMotorIndex, MotorType.kBrushless);
    CANSparkMax frontAngleMotor = new CANSparkMax(22, MotorType.kBrushless);


    // WPI_TalonFX backHeightMotor = new WPI_TalonFX(Constants.backClimberAngleMotorIndex);
    //CANSparkMax backAngleMotor = new CANSparkMax(Constants.backClimberAngleMotorIndex, MotorType.kBrushless);


    public void climberInit(){
        // frontAngleMotor.getEncoder().setPosition(0.0);
        //backAngleMotor.getEncoder().setPosition(0.0);

        // frontHeightMotor.setSelectedSensorPosition(0.0);
        // backHeightMotor.setSelectedSensorPosition(0.0);

        //frontAngleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10); // for holding position and updating faster?
    }
}
