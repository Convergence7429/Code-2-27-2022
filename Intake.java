package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;

public class Intake {

    // 1 spark max for intake
    // 1 spark max for indexer (built in for intake I would say. No reason for another class if possible)

    int ballCount = 1;

    public double intakeIndexerEncoderCount = 200; // encoder Count to get from indexer to under shooter
    
    boolean intaking = false;
    Timer intakeTimer = new Timer();


    
    CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorIndex, MotorType.kBrushless);
    CANSparkMax indexerMotor = new CANSparkMax(Constants.indexerMotorIndex, MotorType.kBrushless);

    boolean colorSensor1Activated = false; // sensor in feeder
    boolean colorSensor2Activated = false; // sensor under shooter

    boolean hasIntakeVelocityBeenMeasured = false;

    // two Color Sensors

    public void intakeInit(){
        // put intake down?
        ballCount = 0; // different for autonomous vs teleop
        colorSensor1Activated = false; // sensor in feeder
        colorSensor2Activated = false; // sensor under shooter
        hasIntakeVelocityBeenMeasured = false;
        intaking = false;
    }

    

    public void intakeOperation(){

        // if(ColorSensor1.getColor()){
            // colorSensor1Activated = true;
        // }

        if((ballCount < 2) && Constants.stick.getRawButtonPressed(1)){
            intaking = true;
            intakeTimer.reset();
            intakeTimer.start();
        }

        if(ballCount == 2){
            intaking = false;
        }

        if(intaking){
            intakeMotor.set(0.6);
            if(intakeTimer.get() > 1.0){ // time for intake motor to speed up
                if((intakeMotor.getEncoder().getVelocity() < (6480/* velocity for 0.6 - 10.0% */)) && !hasIntakeVelocityBeenMeasured){
                    ballCount++;
                    indexerMotor.getEncoder().setPosition(0.0);
                    hasIntakeVelocityBeenMeasured = true;
                }
                if(hasIntakeVelocityBeenMeasured && ballCount == 1){
                    if(indexerMotor.getEncoder().getPosition() < 500){ // number needs to change
                        indexerMotor.set(0.4);
                    } else {
                        hasIntakeVelocityBeenMeasured = false;
                        indexerMotor.set(0.0);
                    }
                }
            }
            
            /*if(!colorSensor1Activated){
                intakeMotor.set(0.5);
                indexerMotor.getEncoder().setPosition(0.0);
            } else {
                if(ballCount == 0){
                    intakeMotor.set(0.0);
                    //indexerMotor.set(Constants.quadraticPositionAndSpeed(0.05, 0.4, intakeIndexerEncoderCount, indexerMotor.getEncoder().getPosition()));
                    indexerMotor.set(0.2);
                    ballCount++;
                    if(indexerMotor.getEncoder().getPosition() > intakeIndexerEncoderCount){
                        indexerMotor.set(0.0);
                        indexerMotor.getEncoder().setPosition(0.0);
                    }
                } else {
                    indexerMotor.set(0.0);
                    indexerMotor.getEncoder().setPosition(0.0);
                    colorSensor1Activated = false;
                }
            }*/
        }
    }

    /*public void teleopIntake1(){

        // if(ColorSensor1.getColor()){
            // colorSensor1Activated = true;
        // }

        if((ballCount < 2) && Constants.stick.getRawButtonPressed(1)){
            intaking = true;
        }

        if(ballCount == 2){
            intaking = false;
        }

        

        if(intaking){
            if(!colorSensor1Activated){
                intakeMotor.set(0.5);
                indexerMotor.getEncoder().setPosition(0.0);
            } else {
                if(ballCount == 0){
                    intakeMotor.set(0.0);
                    //indexerMotor.set(Constants.quadraticPositionAndSpeed(0.05, 0.4, intakeIndexerEncoderCount, indexerMotor.getEncoder().getPosition()));
                    indexerMotor.set(0.2);
                    ballCount++;
                    if(indexerMotor.getEncoder().getPosition() > intakeIndexerEncoderCount){
                        indexerMotor.set(0.0);
                        indexerMotor.getEncoder().setPosition(0.0);
                    }
                } else {
                    indexerMotor.set(0.0);
                    indexerMotor.getEncoder().setPosition(0.0);
                    colorSensor1Activated = false;
                }
            }
        }
    }



            // if(!colorSensor1Activated){
            //     intakeMotor.set(intakeMotorSpeed);
            // } else {

            // }
            
        //     if(colorSensor1Activated){
        //         // reset encoder counts of indexerMotor
        //         // indexerMotor.set(quadraticController(0.05, indexerMotorSpeed, indexerMotor.getEncoder().getPosition(), encoderCount));
        //         if(indexerMotor.getEncoder().getPosition() > encoderCount){
        //             colorSensor1Activated = false;
        //         }
        //     }
        // } else {
        //     intakeMotor.set(0.0);
        // }*/
}