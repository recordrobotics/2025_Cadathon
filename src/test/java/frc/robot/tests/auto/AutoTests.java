package frc.robot.tests.auto;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.getEnabled;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.getMatchTime;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAutonomous;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setMatchTime;
import static utils.Assertions.*;
import static utils.TestRobot.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.PlannedAuto;
import frc.robot.commands.auto.PlannedAuto.AutoSupplier;

class AutoTests {

    boolean periodic() {
        if (getEnabled()) {
            setMatchTime(getMatchTime() - 0.02);
            notifyNewData();
        }
        return true;
    }

    private void testAuto(
            AutoSupplier autoSupplier,
            AllianceStationID allianceStation,
            Constants.FieldStartingLocation startLocation,
            String expectedReefSequence) {
        testUntil(
                () -> getMatchTime() <= 0,
                this::periodic,
                robot -> {
                    /* robot and field setup */

                    setAllianceStationId(allianceStation);
                    setAutonomous(true);
                    setMatchTime(15);
                    notifyNewData();

                    PlannedAuto.setAutoSupplier(autoSupplier);

                    final Pose2d startPose = startLocation.getPose();
                    RobotContainer.drivetrain.getSwerveDriveSimulation().setSimulationWorldPose(startPose);

                    // Odometry reset has to run during periodic to work correctly
                    runFor(2, () -> RobotContainer.poseSensorFusion.setToPose(startPose));

                    // wait two periodic cycles before enable for odometry reset
                    runAfter(2, () -> {
                        setEnabled(true);
                        notifyNewData();
                    });
                },
                () -> {},
                Seconds.of(16));
    }
}
