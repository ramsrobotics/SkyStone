package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Define all the CONSTANTS used for autonomous mode.
 */
public interface CCAuto {
    double DT_TURN_SPEED_LOW = 0.2;
    double DT_TURN_SPEED_HIGH = 0.6;
    int DT_TURN_THRESHOLD_LOW = 1;
    int DT_TURN_THRESHOLD_HIGH = 2;
    double DT_MOVE_LOW = 0.3;
    ElapsedTime runTimeOpMode = new ElapsedTime();

    BoKAutoStatus initSoftware(CCAutoOpMode opMode,
                               CCHardwareBot robot);

    void runSoftware();

    enum BoKAllianceColor {
        BOK_ALLIANCE_RED,
        BOK_ALLIANCE_BLUE
    }

    enum BoKAutoStatus {
        BOK_AUTO_FAILURE,
        BOK_AUTO_SUCCESS
    }
}
