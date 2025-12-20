package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.FollowerBuilderNew;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.24)
            .forwardZeroPowerAcceleration(-33.2)
            .lateralZeroPowerAcceleration(-61.5)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.04, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.95, 0, 0.08, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0,0.000030,0.6,0.01))
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76.25) //84
            .yVelocity(56.40) //65
            .useBrakeModeInTeleOp(true)
            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.45)
            .strafePodX(2.9)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.6, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilderNew(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

}
