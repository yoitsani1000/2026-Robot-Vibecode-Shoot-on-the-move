package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDState;
import frc.robot.subsystems.intake.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Tunnel;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterCommands {
  // if only running right camera
  public static double offset = 0.06;
  public static double efficiencyFactor = 1.06;
  // public double offset=0.0;
  // Cache Pose2d instances for hubs (avoid allocating in tight loops)
  private static final Pose2d RED_HUB_POSE2D =
      new Pose2d(
          Constants.FieldConstants.RED_HUB_POSE3D.getX(),
          Constants.FieldConstants.RED_HUB_POSE3D.getY(),
          new Rotation2d());

  private static final Pose2d BLUE_HUB_POSE2D =
      new Pose2d(
          Constants.FieldConstants.BLUE_HUB_POSE3D.getX(),
          Constants.FieldConstants.BLUE_HUB_POSE3D.getY(),
          new Rotation2d());
  // Cache left/right hub poses for side aiming
  private static final Pose2d RED_LEFT_POSE2D =
      new Pose2d(
          Constants.FieldConstants.RED_LEFT.getX(),
          Constants.FieldConstants.RED_LEFT.getY(),
          new Rotation2d());
  private static final Pose2d RED_RIGHT_POSE2D =
      new Pose2d(
          Constants.FieldConstants.RED_RIGHT.getX(),
          Constants.FieldConstants.RED_RIGHT.getY(),
          new Rotation2d());
  private static final Pose2d BLUE_LEFT_POSE2D =
      new Pose2d(
          Constants.FieldConstants.BLUE_LEFT.getX(),
          Constants.FieldConstants.BLUE_LEFT.getY(),
          new Rotation2d());
  private static final Pose2d BLUE_RIGHT_POSE2D =
      new Pose2d(
          Constants.FieldConstants.BLUE_RIGHT.getX(),
          Constants.FieldConstants.BLUE_RIGHT.getY(),
          new Rotation2d());

  /**
   * Calculates the required ball exit velocity (ft/s) for a given horizontal distance and hood
   * angle, using the same projectile-motion solve as {@link #calculateShooterRPS}. Shared so the
   * shoot-on-the-move flight-time estimate stays consistent with the actual shot.
   *
   * @param xs Horizontal distance to target in feet
   * @param thetaDegrees Hood angle in degrees
   * @return Required ball exit velocity in ft/s, or 0.0 if the shot is not physically achievable
   */
  private static double calculateExitVelocityFps(double xs, double thetaDegrees) {
    double g = 32.2; // Gravity (ft/s^2)
    double thetaRad = Math.toRadians(thetaDegrees);
    double h = 4.11776908; // Target Height (6) - Launch Height (1.88223092)

    double cosTheta = Math.cos(thetaRad);
    double tanTheta = Math.tan(thetaRad);

    double numerator = g * Math.pow(xs, 2);
    double denominator = 2 * Math.pow(cosTheta, 2) * (xs * tanTheta - h);

    if (denominator <= 0) return 0.0; // Distance/angle combination is physically impossible

    return Math.sqrt(numerator / denominator);
  }

  /**
   * Estimates ball flight time (seconds) for a horizontal distance and hood angle, using the
   * horizontal component of the exit velocity computed in {@link #calculateExitVelocityFps}.
   *
   * @param xs Horizontal distance to target in feet
   * @param thetaDegrees Hood angle in degrees
   * @return Flight time in seconds, or 0.0 if the shot is not physically achievable
   */
  public static double estimateFlightTimeSeconds(double xs, double thetaDegrees) {
    double vBall = calculateExitVelocityFps(xs, thetaDegrees);
    if (vBall <= 0) return 0.0;
    double horizontalVel = vBall * Math.cos(Math.toRadians(thetaDegrees));
    if (horizontalVel <= 0) return 0.0;
    return xs / horizontalVel;
  }

  /**
   * Computes a "virtual" hub pose that compensates for robot velocity while shooting on the move.
   * Aiming at this point (and setting flywheel/hood for the distance to it) instead of the real
   * hub corrects for the ball inheriting the robot's velocity at release, and for the robot
   * continuing to travel during the ball's flight.
   *
   * <p>Derivation: the ball's field-relative velocity is robotVelocity + shooterExitVelocity. For
   * it to travel from the robot to the hub T in flight time t:
   * shooterExitVelocity * t = (T - robotPos) - robotVelocity * t, i.e. the shot must be aimed as
   * if the target were at T - robotVelocity * t. Flight time depends on distance to that point,
   * which depends on flight time, so this iterates a few times to converge (it converges fast:
   * flight time changes little between passes).
   *
   * @param robotPose Current robot (or turret) pose
   * @param hubPose Real hub pose
   * @param fieldRelativeSpeeds Robot's current field-relative chassis speeds
   * @param thetaDegrees Hood angle used to estimate flight time (band adjustments near the actual
   *     shot are a close enough approximation for this estimate)
   * @return The virtual hub pose to aim at
   */
  public static Pose2d computeVirtualHubPose(
      Pose2d robotPose, Pose2d hubPose, ChassisSpeeds fieldRelativeSpeeds, double thetaDegrees) {
    double vxMps = fieldRelativeSpeeds.vxMetersPerSecond;
    double vyMps = fieldRelativeSpeeds.vyMetersPerSecond;
    Pose2d virtualHub = hubPose;
    for (int i = 0; i < 3; i++) {
      double xs = getDistanceToHub(robotPose, virtualHub);
      double t = estimateFlightTimeSeconds(xs, thetaDegrees);
      virtualHub =
          new Pose2d(
              hubPose.getX() - vxMps * t, hubPose.getY() - vyMps * t, hubPose.getRotation());
    }
    Logger.recordOutput("Shooter/VirtualHubPose", virtualHub);
    return virtualHub;
  }

  /**
   * Calculates required flywheel RPS for a given hood angle.
   *
   * @param xs Horizontal distance to target in feet
   * @param thetaDegrees Hood angle in degrees (e.g. 65.0)
   * @return Required flywheel speed in Rotations Per Second (RPS)
   */
  public static double calculateShooterRPS(double xs, double thetaDegrees) {
    // 1. Physical Constants
    double wheelDiameter = 4.0 / 12.0; // 4 inch wheel converted to feet

    // 2. Calculate Required Ball Exit Velocity (v)
    double vBall = calculateExitVelocityFps(xs, thetaDegrees); // Linear ft/s
    if (vBall <= 0) return 0.0; // Distance/angle combination is physically impossible

    // 3. Convert Ball Velocity to Wheel RPS
    // For a single-wheel + hood: Wheel Surface Speed = 2 * Ball Velocity
    double wheelSurfaceSpeed = vBall * 2.0;
    double wheelCircumference = Math.PI * wheelDiameter;

    double rps = wheelSurfaceSpeed / wheelCircumference;

    // 4. Recovery/Efficiency Factor (Adjust based on testing)
    // Most FRC shooters lose ~10-15% to slip/compression
    double ef = efficiencyFactor;
    // if (xs > 17) {
    //   ef += 0.18;
    // } else if (xs > 13) {
    //   ef += 0.08;
    // } else if (xs > 9.9) {
    //   ef += 0.06;
    // }

    Logger.recordOutput("Shooter/effiencyFactor", ef);

    Logger.recordOutput("Shooter/calculatedShooterRPS", rps * ef);
    return rps * ef;
  }

  /** Conversion factor from meters to feet. */
  private static final double METERS_TO_FEET = 3.28084;

  /**
   * Returns horizontal distance to the hub (xs) in feet given robot and hub poses (in meters).
   * Extracted so multiple commands can reuse the same calculation in parallel.
   */
  public static double getDistanceToHub(Pose2d robotPose, Pose2d hubPose) {
    // Pose2d field coordinates are in meters; convert to feet for calculateShooterRPS
    double dx = hubPose.getX() - robotPose.getX();
    double dy = hubPose.getY() - robotPose.getY();
    return Math.hypot(dx, dy) * METERS_TO_FEET;
  }

  /** Returns the angle from the robot's heading to the hub, normalized to [-pi, pi]. */
  public static double getAngleRelativeToHub(Pose2d robotPose, Pose2d hubPose) {
    double angleToHub =
        Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX());
    double robotYaw = robotPose.getRotation().getRadians();
    double angleRelative = angleToHub - robotYaw;
    angleRelative += Math.PI / 2;
    // angleRelative -= 3 * Math.PI / 2;
    angleRelative = -Math.atan2(Math.sin(angleRelative), Math.cos(angleRelative)) + offset;
    angleRelative += Math.PI / 2;
    return (angleRelative);
  }

  /** Return the hub pose for the current alliance (red or blue). */
  public static Pose2d getAllianceHubPose() {
    return DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red
        ? RED_HUB_POSE2D
        : BLUE_HUB_POSE2D;
  }

  /** Choose the left/right hub based on robot Y (used by AimToSide). */
  public static Pose2d getSidePose(Pose2d robotPose) {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      if (robotPose.getY() > 4) {
        return RED_LEFT_POSE2D;
      } else {
        return RED_RIGHT_POSE2D;
      }
    } else {
      if (robotPose.getY() < 4) {
        return BLUE_LEFT_POSE2D;
      } else {
        return BLUE_RIGHT_POSE2D;
      }
    }
  }

  /**
   * Returns a Command that continuously aims the turret at the hub using the provided pose
   * suppliers.
   */
  private static Command turretAimCommand(
      Turret turret,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<Pose2d> hubPoseSupplier,
      LEDSubsystem led) {

    return Commands.run(
            () -> {
              Pose2d rp = robotPoseSupplier.get();
              Pose2d hp = hubPoseSupplier.get();
              double angleRelative = getAngleRelativeToHub(rp, hp);
              double rotations = angleRelative / (2 * Math.PI);
              turret.setPositionPID(rotations);
              Logger.recordOutput("test/targetTurretRotations", rotations);
              if (hp == RED_HUB_POSE2D) {
                led.setColor(LEDState.GREEN);
              } else if (hp == BLUE_HUB_POSE2D) {
                led.setColor(LEDState.GREEN);
              } else {
                led.setColor(LEDState.YELLOW);
              }
            },
            turret,
            led)
        .withName("turret aim");
  }

  private static Command turretLogicalAimCommand(
      Turret turret, Supplier<Pose2d> robotPoseSupplier, Supplier<Pose2d> hubPoseSupplier) {

    return Commands.run(
            () -> {
              Pose2d rp = robotPoseSupplier.get();
              Pose2d hp =
                  (rp.getY() < 5.2 || rp.getY() > 10.8) ? hubPoseSupplier.get() : getSidePose(rp);
              double angleRelative = getAngleRelativeToHub(rp, hp);
              double rotations = angleRelative / (2 * Math.PI);
              turret.setPositionPID(rotations);
              Logger.recordOutput("test/targetTurretRotations", rotations);
            },
            turret)
        .withName("turret logical aim");
  }

  public static Command AimEverything(Turret turret, Hood hood, Supplier<Pose2d> poseSupplier) {
    // create a supplier that computes target RPS from the live robot pose

    Supplier<Pose2d> hubPoseSupplier = ShooterCommands::getAllianceHubPose;

    // While shooting, continuously aim the turret and hold the hood at the desired angle.
    // The flywheel shoot command acts as the deadline: when it finishes, aiming/hood are
    // interrupted.
    return Commands.run(
            () -> {
              Pose2d rp = poseSupplier.get();
              Pose2d hp = hubPoseSupplier.get();
              double angleRelative = getAngleRelativeToHub(rp, hp);
              double rotations = angleRelative / (2 * Math.PI);
              turret.setPositionPID(rotations);
            },
            turret)
        // .until(turret.isFinished())

        // hood.setPositionPIDCommand(theta))
        .withName("AimEverythingNoHood");
  }

  public static Command AimEverythingToHub(
      Turret turret, Hood hood, Supplier<Pose2d> poseSupplier, double theta) {
    // create a supplier that computes target RPS from the live robot pose

    Supplier<Pose2d> hubPoseSupplier = ShooterCommands::getAllianceHubPose;

    // While shooting, continuously aim the turret and hold the hood at the desired angle.
    // The flywheel shoot command acts as the deadline: when it finishes, aiming/hood are
    // interrupted.
    return Commands.parallel(
            Commands.run(
                () -> {
                  Pose2d rp = poseSupplier.get();
                  Pose2d hp = hubPoseSupplier.get();
                  double angleRelative = getAngleRelativeToHub(rp, hp);
                  double rotations = angleRelative / (2 * Math.PI);
                  turret.setPositionPID(rotations);
                },
                turret)
            // .until(turret.isFinished())
            ,
            Commands.run(() -> hood.setPositionPID(theta)))
        // hood.setPositionPIDCommand(theta))
        .withName("AimEverythingToHub");
  }

  /**
   * Continuously aims the turret at the hub while compensating for the robot's current
   * field-relative velocity (shoot-on-the-move lead). Does not control the flywheel or hood.
   */
  public static Command AimToHubMoving(
      Turret turret,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier,
      double theta,
      LEDSubsystem led) {
    return Commands.run(
            () -> {
              Pose2d rp = poseSupplier.get();
              Pose2d hp = getAllianceHubPose();
              Pose2d virtualHub =
                  computeVirtualHubPose(rp, hp, fieldSpeedsSupplier.get(), theta);
              double angleRelative = getAngleRelativeToHub(rp, virtualHub);
              double rotations = angleRelative / (2 * Math.PI);
              turret.setPositionPID(rotations);
              Logger.recordOutput("test/targetTurretRotations", rotations);
              led.setColor(LEDState.GREEN);
            },
            turret,
            led)
        .withName("turret aim moving");
  }

  // Shoots while adjusting flywheel speed based on distance
  public static Command ShootFromDistance(
      LEDSubsystem led,
      Flywheel flywheel,
      Hood hood,
      Tunnel tunnel,
      Hopper hopper,
      Intake intake,
      Supplier<Pose2d> poseSupplier,
      double theta) {
    // create a supplier that computes target RPS from the live robot pose
    Supplier<Double> targetRpsSupplier =
        () -> {
          Pose2d rp = poseSupplier.get();
          Pose2d hp = getAllianceHubPose();
          double liveXs = getDistanceToHub(rp, hp);
          Logger.recordOutput("Shooter/Distance", liveXs);

          // theta is 65
          if (liveXs <= 10) {
            hood.setPositionPID(theta - 5);
            return calculateShooterRPS(liveXs, theta - 5);
          } else if (liveXs > 15) {
            hood.setPositionPID(theta + 5);
            return calculateShooterRPS(liveXs, theta + 5);
          } else {
            hood.setPositionPID(theta);
            return calculateShooterRPS(liveXs, theta);
          }
        };

    return Commands.deadline(
            CommandFactory.shootCommand(led, flywheel, tunnel, hopper, intake, targetRpsSupplier))
        .withName("ShootFromDistance");
  }

  /**
   * Shoots while adjusting flywheel speed and hood based on distance to a velocity-compensated
   * virtual hub position, so the shot lands correctly while the robot is driving. Turret aim
   * itself is handled separately by {@link #AimToHubMoving}.
   */
  public static Command ShootFromDistanceMoving(
      LEDSubsystem led,
      Flywheel flywheel,
      Hood hood,
      Tunnel tunnel,
      Hopper hopper,
      Intake intake,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier,
      double theta) {
    Supplier<Double> targetRpsSupplier =
        () -> {
          Pose2d rp = poseSupplier.get();
          Pose2d hp = getAllianceHubPose();
          Pose2d virtualHub = computeVirtualHubPose(rp, hp, fieldSpeedsSupplier.get(), theta);
          double liveXs = getDistanceToHub(rp, virtualHub);
          Logger.recordOutput("Shooter/Distance", liveXs);

          // theta is 65
          if (liveXs <= 10) {
            hood.setPositionPID(theta - 5);
            return calculateShooterRPS(liveXs, theta - 5);
          } else if (liveXs > 15) {
            hood.setPositionPID(theta + 5);
            return calculateShooterRPS(liveXs, theta + 5);
          } else {
            hood.setPositionPID(theta);
            return calculateShooterRPS(liveXs, theta);
          }
        };

    return Commands.deadline(
            CommandFactory.shootCommand(led, flywheel, tunnel, hopper, intake, targetRpsSupplier))
        .withName("ShootFromDistanceMoving");
  }

  public static Command PassFromDistance(
      LEDSubsystem led,
      Flywheel flywheel,
      Hood hood,
      Tunnel tunnel,
      Hopper hopper,
      Supplier<Pose2d> poseSupplier,
      double theta) {
    // create a supplier that computes target RPS from the live robot pose
    Supplier<Double> targetRpsSupplier =
        () -> {
          Pose2d rp = poseSupplier.get();
          Pose2d hp = getAllianceHubPose();
          double liveXs = getDistanceToHub(rp, hp);
          Logger.recordOutput("Shooter/Distance", liveXs);
          hood.setPositionPID(78);
          return 55.0;
        };

    return Commands.deadline(
            CommandFactory.shootCommandNoIntake(led, flywheel, tunnel, hopper, targetRpsSupplier))
        .withName("ShootFromDistance");
  }

  public static Command AimToSide(Turret turret, Supplier<Pose2d> poseSupplier, LEDSubsystem led) {
    Supplier<Pose2d> hubPoseSupplier = () -> getSidePose(poseSupplier.get());
    return turretAimCommand(turret, poseSupplier, hubPoseSupplier, led);
  }

  /**
   * Backwards-compatible overload: aim turret to hub using only turret and a pose supplier. This
   * does not control the flywheel or hood.
   */
  public static Command AimToHub(Turret turret, Supplier<Pose2d> poseSupplier, LEDSubsystem led) {
    Supplier<Pose2d> hubPoseSupplier = ShooterCommands::getAllianceHubPose;
    return turretAimCommand(turret, poseSupplier, hubPoseSupplier, led).withName("turretAimToHub");
  }

  public static Command AimToHubOrSide(Turret turret, Supplier<Pose2d> poseSupplier) {
    Supplier<Pose2d> hubPoseSupplier = ShooterCommands::getAllianceHubPose;
    return turretLogicalAimCommand(turret, poseSupplier, hubPoseSupplier)
        .withName("turretAimToHub");
  }

  public static Pose3d getTurretPose(Supplier<Pose2d> poseSupplier) {
    Pose2d drivePose = poseSupplier.get();
    return new Pose3d(drivePose)
        .plus(
            new Transform3d(
                Inches.of(-8.708), Inches.of(8.299016), Inches.of(18.091), new Rotation3d()));
  }

  public static Pose3d getFlywheelPose(
      Supplier<Pose2d> poseSupplier, Supplier<Double> turretPosSupplier) {
    double turretAngleRads = turretPosSupplier.get() * 2 * Math.PI;
    return getTurretPose(poseSupplier)
        .plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, turretAngleRads)))
        .plus(
            new Transform3d(Inches.of(3.623), Inches.of(0), Inches.of(4.495771), new Rotation3d()));
  }
}
