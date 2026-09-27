package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
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

    public static Translation2d hubPosition()
    {
        if (isBlue())
        {
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }
        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
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
