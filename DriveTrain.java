package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class DriveTrain {

    // 4 drive motors. FL, BL, FR, BR
    CANSparkMax flMotor = new CANSparkMax(Constants.flMotorIndex, MotorType.kBrushless);
    CANSparkMax blMotor = new CANSparkMax(Constants.blMotorIndex, MotorType.kBrushless);
    CANSparkMax frMotor = new CANSparkMax(Constants.frMotorIndex, MotorType.kBrushless);
    CANSparkMax brMotor = new CANSparkMax(Constants.brMotorIndex, MotorType.kBrushless);
    MecanumDrive mecanumDrive = new MecanumDrive(flMotor, blMotor, frMotor, brMotor);

    boolean direction = false; // false: intake is front. true: shooter in front

    final double motorSpeedThresholdTeleop = 0.4;
    final double maxMotorSpeedTeleop = 0.87;
    final double minMotorSpeedEncoders = 0.1;
    final double maxMotorSpeedEncoders = 0.4;

    final static double ENCODERS_PER_REV = 42.0; // encoder counts per revolution of the motor
    final static double GEAR_RATIO = 12.75; // inches // motor spins 12.75 times for wheel to spin once
    final static double wheelRadius = 4.0; // inches
    final static double driveTrainInchesOffset = 0.0; // inches

    boolean isDriverControlEnabled = false;

    public void driveTrainInit(){
        frMotor.setInverted(true);
        brMotor.setInverted(true);
        blMotor.setInverted(false);
        flMotor.setInverted(false);
        // flMotor.setInverted(true);
        // blMotor.setInverted(true);


        resetDriveTrainEncoders();
        
        flMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        frMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        blMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        brMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        direction = false;
    }

    public void driveTrainByControls() {

        if(Constants.stick.getRawAxis(3) < 0.00){
            direction = false; // intake is forward
        } else {
            direction = true; // shooter is forward
        }

        if (isDriverControlEnabled) {
            double ySpeed = Constants.stick.getRawAxis(1);
            double xSpeed = Constants.stick.getRawAxis(0);
            double zSpeed = Constants.stick.getRawAxis(2);

            double yDirectionMaintainer = 1.0;
            double xDirectionMaintainer = 1.0;
            double zDirectionMaintainer = 1.0;

            if ((ySpeed > -motorSpeedThresholdTeleop) && (ySpeed < motorSpeedThresholdTeleop)) { // minimum threshold to go so no interference
                ySpeed = 0.00;
            }

            if ((xSpeed > -motorSpeedThresholdTeleop) && (xSpeed < motorSpeedThresholdTeleop)) {
                xSpeed = 0.00;
            }

            if ((zSpeed > -motorSpeedThresholdTeleop) && (zSpeed < motorSpeedThresholdTeleop)) {
                zSpeed = 0.00;
            }

            if (ySpeed > maxMotorSpeedTeleop) {
                ySpeed = maxMotorSpeedTeleop;
            }

            if (xSpeed > maxMotorSpeedTeleop) {
                xSpeed = maxMotorSpeedTeleop;
            }

            if (zSpeed > maxMotorSpeedTeleop) {
                zSpeed = maxMotorSpeedTeleop;
            }

            if (ySpeed < 0.0) {
                yDirectionMaintainer = -1.0;
            }
            if (xSpeed < 0.0) {
                xDirectionMaintainer = -1.0;
            }
            if (zSpeed < 0.0) {
                zDirectionMaintainer = -1.0;
            }

            //mecanumDrive.driveCartesian(ySpeed, -xSpeed, -zSpeed);
            //mecanumDrive.driveCartesian(ySpeed, xSpeed, zSpeed);

            if(direction){
                mecanumDrive.driveCartesian(0.2 * ySpeed * ySpeed * yDirectionMaintainer, 0.2 * -xSpeed * xSpeed * xDirectionMaintainer,
                0.2 * zSpeed * zSpeed * zDirectionMaintainer); // shooter is front
            } else {
                mecanumDrive.driveCartesian(0.2 * -ySpeed * ySpeed * yDirectionMaintainer, 0.2 * xSpeed * xSpeed * xDirectionMaintainer,
                    0.2 * zSpeed * zSpeed * zDirectionMaintainer); // intake is front
            }

            
        }
    }

    public void driveTrainByInches(double inches, int direction){ // INTAKE ALWAYS FORWARD. can be done with PID?
        // 0 = forward. 1 = back. 2 = left. 3 = right. 4 = turn left. 5 = turn right
        
        //double encoderCounts = 0.0;
        // minus 1.5 (driveTrainInchesOffset) will have to be test for accuracy on different speeds and when robot weight changes
        // we could literally make the encoderCount PositionConversionFactor equate to inches outright. That could
        // just be for the drive train

        if(direction == 0){ // forward
            mecanumDrive.driveCartesian(Constants.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()), 0.0, 0.0);
        }

        if(direction == 1){ // back
            mecanumDrive.driveCartesian(-Constants.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), -flMotor.getEncoder().getPosition()), 0.0, 0.0);
        }

        if(direction == 2){ // left
            mecanumDrive.driveCartesian(0.0, Constants.quadraticPositionAndSpeed(-minMotorSpeedEncoders, 
            -maxMotorSpeedEncoders, inchesToEncoders(inches), frMotor.getEncoder().getPosition()), 0.0);
        }

        if(direction == 3){ // right
            mecanumDrive.driveCartesian(0.0, Constants.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()), 0.0);
        }

        if(direction == 4){ // turn left
            mecanumDrive.driveCartesian(0.0, 0.0, Constants.quadraticPositionAndSpeed(-minMotorSpeedEncoders, 
            -maxMotorSpeedEncoders, inchesToEncoders(inches), frMotor.getEncoder().getPosition()));
        }

        if(direction == 5){ // turn right
            mecanumDrive.driveCartesian(0.0, 0.0, Constants.quadraticPositionAndSpeed(minMotorSpeedEncoders, 
            maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()));
        }


    }

    ///////////////////////////////////////////////////////////////////////
    // useful drivetrain functions
    public void resetDriveTrainEncoders() {
        flMotor.getEncoder().setPosition(0);
        frMotor.getEncoder().setPosition(0);
        blMotor.getEncoder().setPosition(0);
        brMotor.getEncoder().setPosition(0);
    }

    public static double inchesToEncoders(double inches) {
        return (((Math.abs(inches) - driveTrainInchesOffset) / (2.0 * Math.PI * wheelRadius)) * ENCODERS_PER_REV
                * GEAR_RATIO);
    }

    public static double encodersToInches(double encoders) {
        return ((((encoders / GEAR_RATIO) / ENCODERS_PER_REV) * (2.0 * Math.PI * wheelRadius))
                + driveTrainInchesOffset);
    }

    public void stopMotors(){
        flMotor.set(0.0);
        blMotor.set(0.0);
        frMotor.set(0.0);
        brMotor.set(0.0);
    }
    /////////////////////////////////////////////////////////////////////
}
