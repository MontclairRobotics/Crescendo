// import com.pathplanner.lib.auto.AutoBuilder;

// public class Auto {
//     public Auto() {
//         AutoBuilder.configureHolonomic(
//         this::getPose, // Robot pose supplier
//         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
//         this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//         this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
//         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
//                                          new PIDConstants(5.0, 0.0, 0.0),
//                                          // Translation PID constants
//                                          new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
//                                                           swerveDrive.swerveController.config.headingPIDF.i,
//                                                           swerveDrive.swerveController.config.headingPIDF.d),
//                                          // Rotation PID constants
//                                          4.5,
//                                          // Max module speed, in m/s
//                                          swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
//                                          // Drive base radius in meters. Distance from robot center to furthest module.
//                                          new ReplanningConfig()
//                                          // Default path replanning config. See the API for the options here
//         ),
//         () -> {
//           // Boolean supplier that controls when the path will be mirrored for the red alliance
//           // This will flip the path being followed to the red side of the field.
//           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
//           var alliance = DriverStation.getAlliance();
//           return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
//         },
//         this // Reference to this subsystem to set requirements
//                                   );
//     }
// }