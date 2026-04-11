// package frc.robot.subsystems;

// import frc.robot.subsystems.roller.RollerConfig;
// import frc.robot.subsystems.roller.RollerIOSim;
// import frc.robot.subsystems.roller.RollerIOSparkMax;

// public class ClimberConfig extends RollerConfig {
//   public ClimberConfig() {
//     this(true);
//   }

//   public ClimberConfig(boolean real) {
//     name = "ClimberConfig";
//     intakePercent = 0.9; // Need to figure out directions
//     releasePercent = 0.9;
//     if (real) {
//       io = new RollerIOSparkMax(9, 10, false, false, 2 * Math.PI, 2 * Math.PI / 60, 40, 0.5,
// 0.0);
//     } else {
//       io = new RollerIOSim(1, (2.0 * Math.PI / 4096));
//     }
//   }
// }
