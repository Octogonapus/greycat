import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.IDriveEngine
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.BowlerAbstractDevice
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.common.Log
import org.apache.commons.math3.analysis.function.Sin

import java.util.concurrent.Semaphore

public class UnreachableTransformException extends RuntimeException {

    public DHParameterKinematics limb
    public TransformNR target

    public UnreachableTransformException(DHParameterKinematics limb, TransformNR target, String message) {
        super(message)
    }

    public UnreachableTransformException(DHParameterKinematics limb, TransformNR target, Throwable cause) {
        super(cause)
    }

    public UnreachableTransformException(DHParameterKinematics limb, TransformNR target, String message, Throwable cause) {
        super(message, cause)
    }
}

Log.enableSystemPrint(true)

//def dev = DeviceManager.getSpecificDevice("hidDevice")
//if (dev == null) {
//	throw new IllegalStateException("hidDevice was null");
//}
//
//MobileBase base = DeviceManager.getSpecificDevice("MediumKat") as MobileBase
//if (base == null) {
//    throw new IllegalStateException("MediumKat device was null.");
//}
//
//DHParameterKinematics tail = null
//for(DHParameterKinematics l : base.getAllDHChains()) {
//	if(l.getScriptingName().contains("Tail")) {
//		tail = l
//	}
//}
//
//double[] imuDataValues = dev.simple.getImuData() // data kept up to date by device coms thread

//homeLegs(base)
//Thread.sleep(500)
//
//TransformNR fiducialToGlobal = base.getFiducialToGlobalTransform()

//walkBase(base, fiducialToGlobal, new TransformNR(-460, 0, 0, new RotationNR(0, 0, 0)).inverse(), 12, 10, 300)
//walkBase(base, fiducialToGlobal, new TransformNR(0, 0, 0, new RotationNR(0, 75, 0)).inverse(), 10, 10, 300)
/*
walkBase(base, fiducialToGlobal, new TransformNR(480, 0, 0, new RotationNR(0, 0, 0)).inverse(), 15, 10, 300)
homeLegs(base)
Thread.sleep(200)
walkBase(base, fiducialToGlobal, new TransformNR(0, 0, 0, new RotationNR(0, 85, 0)).inverse(), 15, 10, 300)
homeLegs(base)*/
//walkBase(base, fiducialToGlobal, new TransformNR(0, 100, 0, new RotationNR(0, 0, 0)).inverse(), 15, 10, 300)

class SingleBaseEngine {

    private static void homeBase(MobileBase base) {
        base.getAllDHChains().each {
            it.setDesiredTaskSpaceTransform(it.calcHome(), 0)
        }
    }

    /**
     * Solve for a limb's tip position in world space.
     *
     * @param leg The limb to solve for.
     * @param originalTipInLimbSpace The tip position in limb space before the body moved.
     * @param newBodyPose The new body position.
     * @return The new limb position in world space.
     */
    private static TransformNR solveForTipPositionInWorldSpace(
            DHParameterKinematics leg,
            TransformNR originalTipInLimbSpace,
            TransformNR newBodyPose) {
        TransformNR T_tipGlobal = originalTipInLimbSpace
        TransformNR T_fiducialLimb = leg.getRobotToFiducialTransform()
        TransformNR tipInLimbSpace = leg.inverseOffset(T_tipGlobal)
        return newBodyPose.times(T_fiducialLimb).times(tipInLimbSpace)
    }

    /**
     * Compute an interpolation for the legs along a body delta.
     *
     * @param legs The legs.
     * @param originalTipPositions The tip positions the legs started in.
     * @param globalFiducial The current body position.
     * @param baseDelta The body delta.
     * @param numberOfIncrements The number of slices for the interpolation along the delta.
     * @return A list of all the tip positions in world space for all legs in the same order as the
     * legs.
     */
    private static List<List<TransformNR>> computeInterpolation(
            List<DHParameterKinematics> legs,
            List<TransformNR> originalTipPositions,
            TransformNR globalFiducial,
            TransformNR baseDelta,
            int numberOfIncrements) {
        List<List<TransformNR>> points = new ArrayList<List<TransformNR>>(numberOfIncrements)

        // Slice the delta into numberOfIncrements + 1
        for (int i = 0; i <= numberOfIncrements; i++) {
            double scale = i / (double) numberOfIncrements
            List<TransformNR> legPoints = new ArrayList<TransformNR>(legs.size())

            // Add the world space tip transforms to the legPoints list in the same order as the legs
            for (int j = 0; j < legs.size(); j++) {
                legPoints.add(
                        solveForTipPositionInWorldSpace(
                                legs[j],
                                // Use the original tip positions so that the limb does not move in
                                // world space
                                originalTipPositions[j],
                                // Need to scale the delta or else things get weird because the delta
                                // is usually a fairly simple (usually even linear) transform so this
                                // primitive scaling technique works fine
                                globalFiducial.times(baseDelta.scale(scale))
                        )
                )
            }

            points.add(legPoints)
        }

        return points
    }

    /**
     * Follows a list of tip positions for legs.
     *
     * @param legs The legs.
     * @param points The tip positions for all the legs in the same order as the legs.
     * @param timeMs The time to follow the entire list over.
     */
    private static void followTransforms(
            List<DHParameterKinematics> legs,
            List<List<TransformNR>> points,
            long timeMs) {
        long timePerIteration = (long) (timeMs / (double) points.size())

        for (int i = 0; i < points.size(); i++) {
            List<TransformNR> tipPositions = points[i]

            for (int j = 0; j < legs.size(); j++) {
                def leg = legs[j]
                def tipTargetInWorldSpace = tipPositions[j]

                // TODO: Remove this offset
                def foo = tipTargetInWorldSpace.times(new TransformNR(0, 0, 0, new RotationNR()))

                // Only move to the tip target if it is reachable or else the IK blows up
                if (leg.checkTaskSpaceTransform(foo)) {
                    leg.setDesiredTaskSpaceTransform(foo, 0)
                } else {
                    //print("Skipped transform in followTransforms. point=" + i + ", leg=" + j + "\n")
                }
            }

            Thread.sleep(timePerIteration)
        }
    }

    /**
     * Moves the base around while keeping the limbs planted (feet don't translate in world space).
     *
     * @param base The base.
     * @param baseDelta The delta to apply to the base.
     * @param numberOfIncrements The number of slices to interpolate the delta over.
     * @param timeMs The time to complete the movement over.
     */
    private static void moveBaseWithLimbsPlanted(
            MobileBase base,
            TransformNR fiducialToGlobal,
            TransformNR baseDelta,
            int numberOfIncrements,
            long timeMs) {
        def originalTipPositionsInLimbSpace = base.getLegs().collect { leg ->
            leg.getCurrentPoseTarget()
        }

        def points = computeInterpolation(
                base.getLegs(),
                // Use the original tip positions so the feet don't translate in world space
                originalTipPositionsInLimbSpace,
                fiducialToGlobal,
                baseDelta,
                numberOfIncrements
        )

        followTransforms(base.getLegs(), points, timeMs)
    }

    /**
     * Creates a trapezoid profile.
     * @param baseDelta The delta to apply to the base.
     * @param stepHeight The height of one step.
     * @return The profile.
     */
    private static List<TransformNR> createTrapezoidProfile(
            TransformNR baseDelta,
            double stepHeight) {
        TransformNR quarterBaseDelta = baseDelta.scale(1 / 4.0)
        TransformNR quarterBaseDeltaInverted = baseDelta.inverse().scale(1 / 4.0)
        TransformNR twelfthBaseDelta = baseDelta.scale(1 / 12.0)
        return [
                twelfthBaseDelta,
                twelfthBaseDelta,
                twelfthBaseDelta,
                quarterBaseDelta.times(new TransformNR(0, 0, stepHeight, new RotationNR())),
                quarterBaseDeltaInverted,
                quarterBaseDeltaInverted,
                quarterBaseDeltaInverted,
                quarterBaseDeltaInverted,
                quarterBaseDelta.times(new TransformNR(0, 0, -stepHeight, new RotationNR())),
                twelfthBaseDelta,
                twelfthBaseDelta,
                twelfthBaseDelta
        ]
    }

    /**
     * Creates a motion profile for a list of legs. Assumes that the legs start in the home position.
     *
     * @param globalFiducial The position of the base.
     * @param baseDelta The delta to apply to the base.
     * @param limbGroup The limbs to check the profile against.
     * @param stepHeight The height of one step.
     * @return A list of base deltas to follow in order.
     */
    private static List<TransformNR> createLimbTipMotionProfile(
            TransformNR globalFiducial,
            TransformNR baseDelta,
            List<DHParameterKinematics> limbGroup,
            double stepHeight) {
        def profile = createTrapezoidProfile(baseDelta, stepHeight)

        // Assume the legs start homed
        def startingTipPositions = limbGroup.collect { leg ->
            leg.calcHome()
        }

        for (int i = 0; i < profile.size(); i++) {
            TransformNR profileBodyDelta = profile[i]
            // Just calculate the new base position. Don't change modify globalFiducial here.
            TransformNR newBase = globalFiducial.times(profileBodyDelta)

            for (int j = 0; j < limbGroup.size(); j++) {
                def leg = limbGroup[j]
                // Calculate a new tip target for the leg given the new base
                def newTip = solveForTipPositionInWorldSpace(leg, startingTipPositions[j], newBase)

                // Don't continue if the tip target is unreachable
                if (!leg.checkTaskSpaceTransform(newTip)) {
                    //println("New tip:\n" + newTip + "\n")
                    //println("Body delta:\n" + profileBodyDelta + "\n")
                    //println("New body:\n" + globalFiducial.times(profileBodyDelta) + "\n")

                    throw new UnreachableTransformException(
                            leg,
                            newTip,
                            "Unreachable: profile index " + i + ", leg index " + j
                    )
                }

                // Assume the leg went where it was supposed to go
                startingTipPositions[j] = newTip
            }
        }

        return profile
    }

    /**
     * Computes the interpolation transforms between the steps of a profile. Assumes that the legs
     * start in the home position.
     *
     * @param group The legs.
     * @param profile The profile from createLimbTipMotionProfile.
     * @param globalFiducial The body position.
     * @param numberOfIncrements The number of slices to interpolate between each step in the profile.
     * @param startIndex The index to start the profile from.
     * @return A list of all the interpolations of all the profile steps (a list of the results from
     * computeInterpolation).
     */
    private static List<List<List<TransformNR>>> computeInterpolatedGroupProfile(
            List<DHParameterKinematics> group,
            List<TransformNR> profile,
            TransformNR globalFiducial,
            int numberOfIncrements,
            int startIndex) {
        // Final list is profile.size() plus one for the home position
        List<List<List<TransformNR>>> out = new ArrayList<List<List<TransformNR>>>(profile.size() + 1)

        // Assume the legs start homed unless startIndex is nonzero
        def startingTipPositions = group.collect { leg ->
            leg.calcHome()
        }

        // Move through the profile until the startIndex so they start at the correct point in the
        // profile
        for (int i = 0; i < startIndex; i++) {
            def bodyDelta = profile[i]
            def newBody = globalFiducial.times(bodyDelta)

            for (int j = 0; j < group.size(); j++) {
                startingTipPositions[j] = solveForTipPositionInWorldSpace(
                        group[j],
                        startingTipPositions[j],
                        newBody
                )
            }
        }

        out.add(
                computeInterpolation(
                        group,
                        startingTipPositions,
                        globalFiducial,
                        new TransformNR(0, 0, 0, new RotationNR()),
                        1
                )
        )

        // Interpolate between each delta in the profile
        for (int i = 0; i < profile.size(); i++) {
            // Offset i by the startIndex in case it is nonzero
            def adjustedIndex = i + startIndex
            // Wrap the adjustedIndex if it is too large (we wrap around back to the start of the
            // profile)
            if (adjustedIndex >= profile.size()) {
                adjustedIndex -= profile.size()
            }

            def bodyDelta = profile[adjustedIndex]
            // Just calculate the new body. Don't change globalFiducial here.
            def newBody = globalFiducial.times(bodyDelta)

            out.add(
                    computeInterpolation(
                            group,
                            startingTipPositions,
                            globalFiducial,
                            bodyDelta,
                            numberOfIncrements
                    )
            )

            // Assume the legs went where they were supposed to go and save that for the next loop
            for (int j = 0; j < group.size(); j++) {
                startingTipPositions[j] = solveForTipPositionInWorldSpace(
                        group[j],
                        startingTipPositions[j],
                        newBody
                )
            }
        }

        return out
    }

    /**
     * Follows interpolated group profiles for multiple groups by first merging them into one group and
     * one profile.
     *
     * @param groups The leg groups.
     * @param profiles The interpolated profiles (from computeInterpolatedGroupProfile) for the leg
     * groups (in the same order as the groups).
     * @param timeMs The time to follow the entire profile over.
     */
    private static void followInterpolatedGroupProfiles(
            List<List<DHParameterKinematics>> groups,
            List<List<List<List<TransformNR>>>> profiles,
            long timeMs) {
        // Merge the list of groups into one group
        List<DHParameterKinematics> mergedGroup = new ArrayList<DHParameterKinematics>()
        for (int i = 0; i < groups.size(); i++) {
            mergedGroup.addAll(groups[i])
        }

        // Merge the list of profiles into one profile. The innermost list in the profile is the
        // actual tip targets in the same order as the groups, so we need to merge them at that level
        // (so loop all the way in and merge the innermost lists).
        List<List<List<TransformNR>>> mergedProfile = profiles[0]
        for (int i = 1; i < profiles.size(); i++) {
            List<List<List<TransformNR>>> groupProfile = profiles[i]
            for (int j = 0; j < mergedProfile.size(); j++) {
                for (int k = 0; k < mergedProfile[j].size(); k++) {
                    List<TransformNR> mergedProfileTransforms = mergedProfile[j][k]
                    List<TransformNR> groupProfileTransforms = groupProfile[j][k]
                    mergedProfileTransforms.addAll(groupProfileTransforms)
                }
            }
        }

        // The first step in the profile is the home position, move there instantly
        followTransforms(mergedGroup, mergedProfile[0], 0L)

        long timeMsPerProfileStep = (long) (timeMs / (double) (mergedProfile.size() - 1))
        for (int i = 1; i < mergedProfile.size(); i++) {
            followTransforms(mergedGroup, mergedProfile[i], timeMsPerProfileStep)
        }
    }

    /**
     * Moves the base in world space by walking.
     *
     * @param base The base.
     * @param baseDelta The delta to apply to the base.
     * @param stepHeight The height of one step.
     * @param numberOfIncrements The number of slices to interpolate between each step in the generated
     * profiles.
     * @param timeMs The time to complete the entire walk over.
     */
    private static void walkBase(
            MobileBase base,
            TransformNR fiducialToGlobal,
            TransformNR baseDelta,
            double stepHeight,
            int numberOfIncrements,
            long timeMs) {
        def groupA = base.getLegs().subList(0, 2)
        def groupB = base.getLegs().subList(2, 4)

        TransformNR nextBaseDelta = baseDelta
        double nextBaseDeltaScale = 1.0
        double percentOfBaseDeltaCompleted = 0.0

        while (percentOfBaseDeltaCompleted < 1.0) {
            try {
                def profileA = createLimbTipMotionProfile(fiducialToGlobal, nextBaseDelta, groupA, stepHeight)
                def profileB = createLimbTipMotionProfile(fiducialToGlobal, nextBaseDelta, groupB, stepHeight)

//                println("Created motion profiles")

                def interpolatedProfileA = computeInterpolatedGroupProfile(
                        groupA,
                        profileA,
                        fiducialToGlobal,
                        numberOfIncrements,
                        0
                )
                def interpolatedProfileB = computeInterpolatedGroupProfile(
                        groupB,
                        profileB,
                        fiducialToGlobal,
                        numberOfIncrements,
                        7
                )

//                println("Computed interpolated profiles")

                followInterpolatedGroupProfiles(
                        [groupA, groupB],
                        [interpolatedProfileA, interpolatedProfileB],
                        timeMs
                )

                percentOfBaseDeltaCompleted += nextBaseDeltaScale
            } catch (UnreachableTransformException ignored) {
                nextBaseDeltaScale *= 0.75
//                print("New scale: " + nextBaseDeltaScale + "\n")

                if (nextBaseDeltaScale < 0.1) {
                    throw new IllegalStateException("Scale became too small.")
                }

                if (percentOfBaseDeltaCompleted + nextBaseDeltaScale < 1.0) {
                    // The next delta will not move further than the original delta, so we can follow it
                    nextBaseDelta = baseDelta.scale(nextBaseDeltaScale)
                } else {
                    // The next delta will move farther than the original delta, so we only need to follow the remaining distance
                    nextBaseDelta = baseDelta.scale(1.0 - percentOfBaseDeltaCompleted)
                }
            }
        }
    }

    private MobileBase mobileBase
    private Thread thread
    private TransformNR fiducialToGlobal = mobileBase.getFiducialToGlobalTransform()

    private Semaphore semaphore = new Semaphore(1)
    private TransformNR lastTransform
    private double lastSeconds
    private long lastCallTime

    def dev = DeviceManager.getSpecificDevice("hidDevice")
    double[] imuDataValues
    double kTilt_stepLength = -4
    double kTilt_stepHeight = 2
    double kTiltRate_stepLength = -0
    double maxStepHeight = 35
    double minStepHeight = 10
    boolean fellOver = false

    SingleBaseEngine(MobileBase mobileBase) {
        this.mobileBase = mobileBase

        if (dev == null) {
            throw new IllegalStateException("hidDevice was null")
        }

        imuDataValues = dev.simple.getImuData() // data kept up to date by device coms thread
    }

    void DriveArc(TransformNR transformNR, double seconds) {
        // We get a velocity made up of a delta and a time
        // This method sets the target velocity at some high frequency
        // If the time specified here ever elapses without another call to "refresh" it, then the
        //  robot must stop
        // Otherwise, the robot should continue traveling along the velocity
        try {
            semaphore.acquire()

            lastTransform = transformNR
            lastSeconds = seconds
            lastCallTime = System.currentTimeMillis()
//            println("Set lastCallTime=" + lastCallTime)

            // Recreate the thread if it expired
            if (thread == null) {
//                println("Creating new thread")
                thread = createThread()
                thread.start()
            }
        } catch (InterruptedException ignored) {
            println("SingleBaseEngine: Failed to acquire semaphore.")
            Thread.currentThread().interrupt()
        } finally {
            semaphore.release()
        }
    }

    private Thread createThread() {
        return new Thread({
//            println("New thread running")
            TransformNR velocity = new TransformNR()

            while (!Thread.currentThread().isInterrupted()) {
//                println("Loop")
                // Get the latest DriveArc data
                try {
                    semaphore.acquire()
//                    println("Got semaphore")

                    // Check the last time DriveArc was called
                    if (System.currentTimeMillis() - lastCallTime > lastSeconds * 1000) {
                        // No more updates. Stop the robot and exit
//                        println("Stopping and exiting")
                        homeBase(mobileBase)
                        break
                    }

                    // Scale the desired velocity to a movement over 300ms
                    velocity = scaleFT(lastTransform.inverse(), 0.3d / lastSeconds)
                } catch (InterruptedException ignored) {
                    println("SingleBaseEngine Thread: Failed to acquire semaphore.")
                } finally {
                    semaphore.release()
                }

                double tilt = imuDataValues[10]
                double tiltRate = imuDataValues[4]

                if (fellOver) {
                    // We fell over
                    if (Math.abs(tilt) > 5) {
                        // We haven't gotten back up yet
                    } else {
                        // We have gotten back up
                        fellOver = false
                    }
                } else {
                    // We did not fall over (yet)
                    if (Math.abs(tilt) > 20) {
                        // Actually, we did fall over
                        fellOver = true
                    } else if (Math.abs(tilt) > 5) { // + tiltRate * 0.1
                        // We are going to fall over
                        double stepLength = kTilt_stepLength * tilt
                        //+ kTiltRate_stepLength * tiltRate
                        double stepHeight = kTilt_stepHeight * Math.abs(tilt) + 10

                        if (stepHeight > maxStepHeight) {
                            stepHeight = maxStepHeight
                        } else if (stepHeight < minStepHeight) {
                            stepHeight = minStepHeight
                        }

//            walkBase(
//                    base,
//                    fiducialToGlobal,
//                    new TransformNR(0, stepLength, 0, new RotationNR(0, 0, 0)).inverse(),
//                    stepHeight,
//                    10,
//                    200
//            )

                        TransformNR balanceTransform = new TransformNR(0, stepLength, 0, new RotationNR(0, 0, 0)).inverse()

                        // println("Walking " + velocity)
                        walkBase(mobileBase, fiducialToGlobal, velocity.times(balanceTransform), stepHeight, 10, 300)
                    } else {
                        // We are not going to fall over
                        walkBase(mobileBase, fiducialToGlobal, velocity, 15, 10, 300)
                    }
                }
            }

//            println("Exited thread")
            thread = null
        })
    }

    private static scaleFT(TransformNR transformNR, double ratio) {
        double tilt = Math.toDegrees(transformNR.getRotation().getRotationTilt() * ratio)
        double az = Math.toDegrees(transformNR.getRotation().getRotationAzimuth() * ratio)
        double ele = Math.toDegrees(transformNR.getRotation().getRotationElevation() * ratio)
        return new TransformNR(transformNR.getX() * ratio, transformNR.getY() * ratio, transformNR.getZ() * ratio, new RotationNR(tilt, az, ele))
    }
}

class MyDriveEngine implements IDriveEngine {

    private Map<MobileBase, SingleBaseEngine> engines = new HashMap<MobileBase, SingleBaseEngine>()

    @Override
    void DriveArc(MobileBase mobileBase, TransformNR transformNR, double seconds) {
        if (!engines.containsKey(mobileBase)) {
            engines.put(mobileBase, new SingleBaseEngine(mobileBase))
        }

        engines.get(mobileBase).DriveArc(transformNR, seconds)
    }

    @Override
    void DriveVelocityStraight(MobileBase mobileBase, double v) {
        throw new UnsupportedOperationException("Not implemented")
    }

    @Override
    void DriveVelocityArc(MobileBase mobileBase, double v, double v1) {
        throw new UnsupportedOperationException("Not implemented")
    }
}

return new MyDriveEngine()
