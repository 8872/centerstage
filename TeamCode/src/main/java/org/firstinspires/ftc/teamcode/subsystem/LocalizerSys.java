package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.MB1242;

public class LocalizerSys extends SubsystemBase {
    private MB1242 fl,fr, bl;

    public LocalizerSys(MB1242 fl, MB1242 fr, MB1242 bl) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
    }

    public String getPose() {
        return "fl: " + fl.getDistance(DistanceUnit.INCH) + "\nfr: " + fr.getDistance(DistanceUnit.INCH) + "\nbl: " + bl.getDistance(DistanceUnit.INCH);
    }

}
