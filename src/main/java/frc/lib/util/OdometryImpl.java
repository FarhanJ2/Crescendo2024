// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.Swerve;

public class OdometryImpl extends SubsystemBase {
  /** Creates a new OdometryImpl. */
  Swerve s_Swerve;

  public OdometryImpl(Swerve s_Swerve) {
      this.s_Swerve = s_Swerve;
  }
 
  public double getDistance(Pose2d target) {
    //changed from getRelativePose()
      double distance = s_Swerve.getRelativePose().getTranslation().getDistance(target.getTranslation());
      return distance;
  }

  public double getPivotAngle(DriverStation.Alliance alliance) {
    double base;
    double height = Constants.StructureConstants.speakerHeight;
       
    if (alliance == DriverStation.Alliance.Red) {
        base = getDistance(Constants.RedTeamPoses.redSpeakerPose);
 
    }

    else {
        base = getDistance(Constants.BlueTeamPoses.blueSpeakerPose);
    }
        
    return Math.atan(height / base); 


  }

  //This is assuming that the robot is directly facing the target object
  public double getTurnAngle(Pose2d target, double robotAngle) {
      double tx = target.getX();
      double ty = target.getY();
      double rx = s_Swerve.getPose().getX();
      double ry = s_Swerve.getPose().getY();

      double requestedAngle = Math.atan((ty - ry) / (tx - rx)) * (180/ Math.PI);
      double calculatedAngle = (180 - robotAngle + requestedAngle);

      return ((calculatedAngle + 180) % 360) - 180;
  }

  public double getVisionPoseError(Limelight limelight) {
    if(limelight == null) return -1;
      Pose2d predictedPose = limelight.getVisionPredictedRobotPose(); 
      if (predictedPose != null) {
         return Math.abs(s_Swerve.getPose().getTranslation().getDistance(predictedPose.getTranslation()));
      }
      
      return -1;
  }

  public boolean isValidVisionMeasurement(Limelight limelight) {
    if(limelight == null) return false;
      Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
      //deleted the predictedPose = null check 
      if (predictedPose != null && (predictedPose.getX() != 0 && predictedPose.getY() != 0)) {
          if ((limelight.getTagArea() > LimelightConstants.minAreaOfTag) || limelight.getNumberOfTagsInView() > 1) {
              //Newly added if it doesn't work take out
              
              return true;
              
          }
      }

      return false;
  }

  public Pose2d getVisionMeasurementWithoutYaw(Limelight limelight) {
    if(limelight == null) return null;
      //added variable for predicted pose instead of calling function directly
      Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
      if (isValidVisionMeasurement(limelight) && predictedPose != null) {
          //.out.println("ADDED VISION MEASUREMENT");
          return new Pose2d(predictedPose.getTranslation(), s_Swerve.getHeading());
      }
 
      return null;
  } 

   
  public Pose2d getVisionMeasurement(Limelight limelight) {
    if(limelight == null) return null;
      Pose2d predictedPose = limelight.getVisionPredictedRobotPose();
      if (isValidVisionMeasurement(limelight) && predictedPose != null) {
          return predictedPose;
      }

      return null;
  }
  
  // Angle offset should be passed in as degrees
  public Vector<N3> createStdDevs(double n1, double n2, double angleOffset) {
      return VecBuilder.fill(n1, n2, Units.degreesToRadians(angleOffset));
  }


  @Override
  public void periodic() {
      // This method will be called once per scheduler run
      // newly added
      // SmartDashboard.putNumber("Vision Pose Error Limelight Front", getVisionPoseError(s_Swerve.limelightShooter));
      // SmartDashboard.putNumber("Vision Pose Error Limelight Back", getVisionPoseError(s_Swerve.limelightArm));

      SmartDashboard.putNumber(
        (RobotContainer.alliance == DriverStation.Alliance.Red) ? "Red speaker distance" : "Blue speaker distance", 
        ((RobotContainer.alliance == DriverStation.Alliance.Red) ? getDistance(Constants.RedTeamPoses.redSpeakerPose) : getDistance(Constants.BlueTeamPoses.blueSpeakerPose))
      );

      SmartDashboard.putNumber("Calculated Angle from Odometry", getPivotAngle(RobotContainer.s_Swerve.alliance));
    }
}