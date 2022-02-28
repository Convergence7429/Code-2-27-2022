package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

  static Shooter shooter = new Shooter();
  static DriveTrain drive = new DriveTrain();
  static Intake intake = new Intake();
  static Climber climber = new Climber();
  int autoStage = 0;
  // CANSparkMax canSparkMax = new CANSparkMax(15, MotorType.kBrushless);
  Timer timer = new Timer();

  

  ///////////////////////////////////////////////////////
  // Robot

  @Override
  public void robotInit() {
    drive.isDriverControlEnabled = true;
    drive.driveTrainInit();
  }

  @Override
  public void robotPeriodic() {
    
  }

  ////////////////////////////////////////////////////////
  // Autonomous

  @Override
  public void autonomousInit() {
    drive.isDriverControlEnabled = false;
    drive.resetDriveTrainEncoders();
    timer.reset();
    timer.start();
    autoStage = 0;
  }

  @Override
  public void autonomousPeriodic() {
    drive.driveTrainByInches(50, 0);
    /*
    switch(autoStage)
    {
      case 0 :
      {
        drive.resetDriveTrainEncoders();
        timer.reset();
        autoStage = 1;

      }
      break;

      case 1 :
      {
        drive.driveTrainByInches(47, 1);
        if(timer.get() > 5.0)
        {
          autoStage = 2;
          timer.reset();
          drive.resetDriveTrainEncoders();
        }
      }
      break;

      case 2 :
      {
       //drive.driveTrainByInches(250, 3); 
      }
      break;
    }*/


  }

  ///////////////////////////////////////////////////////////
  // Tele-operated

  double shooterSpeed = 0.05;
  boolean canSetPosition = true; // front climber angle
  double frontClimberAnglePosition = 0.00;
  boolean shooterActivated = false;
  boolean canShoot = false;

  @Override
  public void teleopInit() {
    //System.out.println("TELEOP INIT CALLED");
    intake.intakeInit();
    drive.isDriverControlEnabled = true;
    shooter.shooterInit();
    shooterSpeed = 0.00;
    shooterActivated = true;
    //shooter.testVelocity(7 + 11.125 + 33.875);
    //shooter.calculatedAngle = 50.0*Math.PI/180.0;
    // distance from shooter to front of robot (no bumper (CHANGE LATER) 
    // + distance from fender to center of hub + distance from front of robot to fender
    timer.reset();
    timer.start();
    canShoot = false;
    shooter.masterShooterMotor.set(ControlMode.PercentOutput, 0.00);
    //shooter.testVelocity(shooter.getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
  }

  
  

  @Override
  public void teleopPeriodic() {
    drive.driveTrainByControls();
    // System.out.println(intake.intakeMotor.getEncoder().getVelocity()); // 6500
    // System.out.println("Shooter speed: " + shooterSpeed);
    intake.intakeOperation();
    shooter.shooterTeleop();

    //System.out.println(shooter.getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    // System.out.println(shooter.calculatedVelocity);
    // System.out.println(shooter.calculatedAngle);
    //drive.driveTrainByControls();
    
    
    //System.out.println("Timer: " + timer.get());
    /*System.out.println(shooter.calculatedAngle);
    System.out.println(shooter.calculatedVelocity);
    System.out.println(shooter.hoodMotor.getEncoder().getPosition());
    //System.out.println(shooter.shooterWheelLinearVelocityToMotorVelocity(311));
    System.out.println("Sensor velocity: " + shooter.masterShooterMotor.getSelectedSensorVelocity());

    System.out.println(canShoot);

    if(timer.get() > 5.0){
      System.out.println(shooter.shooterWheelLinearVelocityToMotorVelocity(shooter.calculatedVelocity));
      if(shooter.hoodMotor.getEncoder().getPosition() < ((90-(shooter.calculatedAngle*180/Math.PI)))){
        shooter.hoodMotor.set(0.1);
      } else {
        shooter.hoodMotor.set(0.0);
        shooter.hoodMotor.getPIDController().setReference(shooter.calculatedAngle*180/Math.PI, ControlType.kPosition);
        shooter.masterShooterMotor.set(ControlMode.PercentOutput, (shooter.shooterWheelLinearVelocityToMotorVelocity(shooter.calculatedVelocity) / 18850.6));
        if(shooter.masterShooterMotor.getSelectedSensorVelocity() > shooter.shooterWheelLinearVelocityToMotorVelocity(shooter.calculatedVelocity) - 500.0){
          canShoot = true;
        } else {
          canShoot = false;
        }
      }
    } else {
      shooter.hoodMotor.set(0.0);
    }

    if(canShoot){
      if(Constants.stick.getRawButton(2)){
        intake.indexerMotor.set(-0.4);
      } else {
        intake.indexerMotor.set(0.0);
      }
    }*/

    //shooter.masterShooterMotor.set(ControlMode.Velocity, 9600.0);
    //System.out.println(shooter.masterShooterMotor.getSelectedSensorVelocity());

    // if(Constants.stick.getRawButton(1)){
    //   shooter.hoodMotor.getPIDController().setReference(40, ControlType.kPosition);
    // }


    // //drive.driveTrainByControls();

    
    // System.out.println(shooter.hoodMotor.getInverted());
    // System.out.println("intakeMotor: " + intake.intakeMotor.get());
    // System.out.println("Shooter set speed: " + shooterSpeed);
    // System.out.println("Shooter velocity: " + shooter.masterShooterMotor.getSelectedSensorVelocity());
    //System.out.println("Hood Position: " + shooter.hoodMotor.getEncoder().getPosition()); // also measure change in degrees if possible
    // System.out.println("Shooter activated: " + shooterActivated);
    // //System.out.println("Front Climber Position: " + climber.frontAngleMotor.getEncoder().getPosition());
    // System.out.println("*************************************************************");
    

    if(Constants.stick.getRawButtonPressed(7)){
      shooterActivated = !shooterActivated;
    }
    if(Constants.stick.getRawButtonPressed(11)){ // adjust shooter speed
      shooterActivated = true;
      shooterSpeed -= 0.05;
    }
    if(Constants.stick.getRawButtonPressed(12)){
      shooterActivated = true;
      shooterSpeed += 0.05;
    }
    if(shooterActivated){
      shooter.masterShooterMotor.set(ControlMode.PercentOutput, shooterSpeed);
    } else {
      shooter.masterShooterMotor.set(0.0);
    }

    if(Constants.stick.getRawButton(2)){ // run indexer
      intake.indexerMotor.set(-0.4);
    } else {
      intake.indexerMotor.set(0.0);
    }

    if(Constants.stick.getRawButton(1)){ // run intake wheels
      intake.intakeMotor.set(-0.6);
    } else {
      intake.intakeMotor.set(0.0);
    }

    if(Constants.stick.getRawButton(3)){ // adjust hood
      shooter.hoodMotor.set(-0.2);
    } else if (Constants.stick.getRawButton(4)){
      shooter.hoodMotor.set(0.2);
    } else {
      shooter.hoodMotor.set(0.0);
    }

      // if(Constants.stick.getRawButton(5)){ // adjust front climber angle // (Really intake)
      //   climber.frontAngleMotor.set(-0.075);
      // //climber.frontAngleMotor.set(-0.1);
      // //climber.frontAngleMotor.getPIDController().setReference(0.1, ControlType.kDutyCycle);
      //   //climber.frontAngleMotor.getPIDController().setReference(frontClimberAnglePosition - 2, ControlType.kPosition); // setting reference position further
      //   canSetPosition = true;
      // } else if (Constants.stick.getRawButton(6)){
      // //climber.frontAngleMotor.set(0.0);
      //   climber.frontAngleMotor.set(0.075);
      //   //climber.frontAngleMotor.getPIDController().setReference(frontClimberAnglePosition + 2, ControlType.kPosition); // setting reference position further
      //   canSetPosition = true;
      // } else {
      //   climber.frontAngleMotor.set(0.0);
      //   if(canSetPosition){
      //     frontClimberAnglePosition = climber.frontAngleMotor.getEncoder().getPosition();
      //     canSetPosition = false;
      //   }
      //   climber.frontAngleMotor.getPIDController().setReference(frontClimberAnglePosition, ControlType.kPosition);
      // }

    // //   // does this keep you from moving again?
    
  
  }

    
    

  /////////////////////////////////////////////////////////////
  // Test

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }

  //////////////////////////////////////////////////////////////
  // Disabled

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
    climber.frontAngleMotor.getPIDController().setReference(frontClimberAnglePosition, ControlType.kPosition);
  }
}