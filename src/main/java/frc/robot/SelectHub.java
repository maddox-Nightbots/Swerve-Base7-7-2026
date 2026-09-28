package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.TurretConstants;

public class SelectHub
{
    private static boolean isBlue()
    {
        final Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == Alliance.Blue;
    }

    public static Translation2d hubPosition(Pose2d Pose) {
        // get the alliance from DS
        final Optional<Alliance> alliance = DriverStation.getAlliance();

        // if on blue
        if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
            // if in turret in alliance zone auto aim at hub
            if (Units.metersToInches(Pose.getX()) < 182.105) {
                return new Translation2d(Inches.of(182.105), Inches.of(158.845));

                // if turret in bottom half of field aim at bottom trench
            } else if (Units.metersToInches(Pose.getY()) < 158.845) {
                return new Translation2d(Inches.of(182.105), Inches.of(120));

                // if turret in upper half aim at top trench
            } else {
                return new Translation2d(Inches.of(182.105), Inches.of(198));
            }
        }

        // if on red
        else {
            // if in turret in alliance zone auto aim at hub
            if (Units.metersToInches(Pose.getX()) > 469.11) {
                return new Translation2d(Inches.of(469.11), Inches.of(158.845));

                // if turret in bottom half of field aim at bottom trench
            } else if (Units.metersToInches(Pose.getY()) < 158.845) {
                return new Translation2d(Inches.of(469.11), Inches.of(120));

                // if turret in upper half aim at top trench
            } else {
                return new Translation2d(Inches.of(469.11), Inches.of(198));
            }
        }
    }

    /** True if the tag is on our alliance's hub. Checks the alliance every call. */
    public static boolean isOurHubTag(int fiducialId)
    {
        for (int id : isBlue() ? TurretConstants.kBlueHubTagIds : TurretConstants.kRedHubTagIds)
        {
            if (id == fiducialId)
            {
                return true;
            }
        }
        return false;
    }
}
