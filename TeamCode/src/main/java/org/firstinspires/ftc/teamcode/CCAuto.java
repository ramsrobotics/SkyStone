package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Krishna Saxena on 10/14/2016.
 * Define all the CONSTANTS used for autonomous mode.
 */
public interface CCAuto
{
    public enum BoKAllianceColor {
        BOK_ALLIANCE_RED,
        BOK_ALLIANCE_BLUE
    }

    public enum BoKAutoStatus {
        BOK_AUTO_FAILURE,
        BOK_AUTO_SUCCESS
    }

    public static double DT_TURN_SPEED_LOW  = 0.2;
    public static double DT_TURN_SPEED_HIGH = 0.6;
    public static int DT_TURN_THRESHOLD_LOW = 1;
    public static int DT_TURN_THRESHOLD_HIGH = 2;
    public static double DT_MOVE_LOW = 0.3;

    public static ElapsedTime runTimeOpMode = new ElapsedTime();
    public BoKAutoStatus initSoftware(CCAutoOpMode opMode,
                                      CCHardwareBot robot);

    public void runSoftware();
}
