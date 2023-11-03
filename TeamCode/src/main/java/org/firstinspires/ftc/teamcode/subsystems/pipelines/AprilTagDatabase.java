package org.firstinspires.ftc.teamcode.subsystems.pipelines;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

public class AprilTagDatabase {

    public static AprilTagLibrary getLibrary(){
        return new AprilTagLibrary.Builder()
                .addTag(
                        1,
                        "Blue 1",
                        2,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        Quaternion.identityQuaternion()

                )
                .addTag(
                        2,
                        "Blue 2",
                        2,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        Quaternion.identityQuaternion()

                )
                .addTag(
                        3,
                        "Blue 3",
                        2,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        Quaternion.identityQuaternion()

                )
                .addTag(
                        4,
                        "Red 1",
                        2,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        Quaternion.identityQuaternion()

                )
                .addTag(
                        5,
                        "Red 2",
                        2,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        Quaternion.identityQuaternion()

                )
                .addTag(
                        6,
                        "Red 3",
                        2,
                        new VectorF(0, 0, 0),
                        DistanceUnit.INCH,
                        Quaternion.identityQuaternion()

                )

                .build();

    }
}
