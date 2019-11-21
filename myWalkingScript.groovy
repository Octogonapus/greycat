import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.common.Log

void homeLegs(MobileBase base) {
    base.getLegs().each {
        it.setDesiredTaskSpaceTransform(it.calcHome(), 0)
    }
}

TransformNR solveForTipPositionInWorldSpace(
        DHParameterKinematics leg,
        TransformNR originalTipInLimbSpace,
        TransformNR newBodyPose) {
    def T_tipGlobal = originalTipInLimbSpace
    def T_fiducialLimb = leg.getRobotToFiducialTransform()
    def tipInLimbSpace = leg.inverseOffset(T_tipGlobal)
    return newBodyPose.times(T_fiducialLimb).times(tipInLimbSpace)
}

List<List<TransformNR>> computeInterpolation(
        List<DHParameterKinematics> legs,
        List<TransformNR> originalTipPositions,
        TransformNR globalFiducial,
        TransformNR baseDelta,
        int numberOfIncrements) {
    List<List<TransformNR>> points = new ArrayList<List<TransformNR>>(numberOfIncrements)

    for (int i = 0; i <= numberOfIncrements; i++) {
        double scale = i / (double) numberOfIncrements
        List<TransformNR> legPoints = new ArrayList<TransformNR>(legs.size())

        for (int j = 0; j < legs.size(); j++) {
            legPoints.add(
                    solveForTipPositionInWorldSpace(
                            legs[j],
                            originalTipPositions[j],
                            globalFiducial.times(baseDelta.scale(scale))
                    )
            )
        }

        points.add(legPoints)
    }

    return points
}

void followTransforms(List<DHParameterKinematics> legs, List<List<TransformNR>> points, long timeMs) {
    long timePerIteration = (long) (timeMs / (double) points.size())
    for (int i = 0; i < points.size(); i++) {
        for (int j = 0; j < legs.size(); j++) {
            def leg = legs[j]

            def tipTargetInWorldSpace = points[i][j]

            if (leg.checkTaskSpaceTransform(tipTargetInWorldSpace)) {
                leg.setDesiredTaskSpaceTransform(tipTargetInWorldSpace, 0)
            }
        }

        Thread.sleep(timePerIteration)
    }
}

void moveBaseWithLimbsPlanted(
        MobileBase base,
        TransformNR baseDelta,
        int numberOfIncrements,
        int timeMs) {
    def originalTipPositionsInLimbSpace = base.getLegs().collect { leg ->
        leg.getCurrentPoseTarget()
    }

    def points = computeInterpolation(
            base.getLegs(),
            originalTipPositionsInLimbSpace,
            base.getFiducialToGlobalTransform(),
            baseDelta,
            numberOfIncrements
    )

    followTransforms(base.getLegs(), points, timeMs)
}

List<TransformNR> createLimbTipMotionProfile(
        TransformNR globalFiducial,
        TransformNR baseDelta,
        List<DHParameterKinematics> limbGroup,
        double stepHeight) {
    TransformNR quarterBaseDelta = baseDelta.scale(1 / 4.0)
    TransformNR quarterBaseDeltaInverted = baseDelta.inverse().scale(1 / 4.0)
    def profile = [
            quarterBaseDelta,
            quarterBaseDelta.times(new TransformNR(0, 0, stepHeight, new RotationNR())),
            quarterBaseDeltaInverted,
            quarterBaseDeltaInverted,
            quarterBaseDeltaInverted,
            quarterBaseDeltaInverted,
            quarterBaseDelta.times(new TransformNR(0, 0, -stepHeight, new RotationNR())),
            quarterBaseDelta
    ]

    def startingTipPositions = limbGroup.collect { leg ->
        leg.calcHome()
    }

    for (int i = 0; i < profile.size(); i++) {
        TransformNR profileBodyDelta = profile[i]
        TransformNR newBase = globalFiducial.times(profileBodyDelta)

        for (int j = 0; j < limbGroup.size(); j++) {
            def leg = limbGroup[j]
            def newTip = solveForTipPositionInWorldSpace(leg, startingTipPositions[j], newBase)

            if (!leg.checkTaskSpaceTransform(newTip)) {
                println("New tip:\n" + newTip + "\n")
                println("Body delta:\n" + profileBodyDelta + "\n")
                println("New body:\n" + globalFiducial.times(profileBodyDelta) + "\n")

                throw new UnsupportedOperationException(
                        "Unreachable: profile index " + i + ", leg index " + j
                )
            }

            startingTipPositions[j] = newTip
        }
    }

    return profile
}

List<List<List<TransformNR>>> computeInterpolatedGroupProfile(
        List<DHParameterKinematics> group,
        List<TransformNR> profile,
        TransformNR globalFiducial,
        int numberOfIncrements,
        int startIndex) {
    List<List<List<TransformNR>>> out = new ArrayList<List<List<TransformNR>>>(profile.size() + 1)

    def startingTipPositions = group.collect { leg ->
        leg.calcHome()
    }

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

    for (int i = 0; i < profile.size(); i++) {
        def adjustedIndex = i + startIndex
        if (adjustedIndex >= profile.size()) {
            adjustedIndex -= profile.size()
        }

        def bodyDelta = profile[adjustedIndex]
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

void followInterpolatedGroupProfiles(
        List<List<DHParameterKinematics>> groups,
        List<List<List<List<TransformNR>>>> profiles,
        long timeMs) {
    List<DHParameterKinematics> mergedGroup = new ArrayList<DHParameterKinematics>()
    for (int i = 0; i < groups.size(); i++) {
        mergedGroup.addAll(groups[i])
    }

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

    followTransforms(mergedGroup, mergedProfile[0], 0L)

    long timeMsPerProfileStep = (long) (timeMs / (double) (mergedProfile.size() - 1))
    for (int i = 1; i < mergedProfile.size(); i++) {
        followTransforms(mergedGroup, mergedProfile[i], timeMsPerProfileStep)
    }
}

void walkBase(
        MobileBase base,
        TransformNR baseDelta,
        double stepHeight,
        int numberOfIncrements,
        int timeMs) {
    def groupA = base.getLegs().subList(0, 2)
    def groupB = base.getLegs().subList(2, 4)

    // TODO: Split baseDelta if it is impossible
    def globalFiducial = base.getFiducialToGlobalTransform()
    def profileA = createLimbTipMotionProfile(globalFiducial, baseDelta, groupA, stepHeight)
    def profileB = createLimbTipMotionProfile(globalFiducial, baseDelta, groupB, stepHeight)

    def interpolatedProfileA = computeInterpolatedGroupProfile(
            groupA,
            profileA,
            globalFiducial,
            numberOfIncrements,
            0
    )
    def interpolatedProfileB = computeInterpolatedGroupProfile(
            groupB,
            profileB,
            globalFiducial,
            numberOfIncrements,
            4
    )

    followInterpolatedGroupProfiles(
            [groupA, groupB],
            [interpolatedProfileA, interpolatedProfileB],
            timeMs
    )
}

Log.enableSystemPrint(true)

MobileBase base = DeviceManager.getSpecificDevice("MediumKat") as MobileBase
if (base == null) {
    throw new IllegalStateException("MediumKat device was null.");
}

homeLegs(base)
Thread.sleep(500)

walkBase(base, new TransformNR(30, 0, 0, new RotationNR(0, 0, 0)).inverse(), 10, 10, 1000)
walkBase(base, new TransformNR(30, 0, 0, new RotationNR(0, 0, 0)).inverse(), 10, 10, 1000)
walkBase(base, new TransformNR(30, 0, 0, new RotationNR(0, 0, 0)).inverse(), 10, 10, 1000)
walkBase(base, new TransformNR(30, 0, 0, new RotationNR(0, 0, 0)).inverse(), 10, 10, 1000)
walkBase(base, new TransformNR(30, 0, 0, new RotationNR(0, 0, 0)).inverse(), 10, 10, 1000)
walkBase(base, new TransformNR(30, 0, 0, new RotationNR(0, 0, 0)).inverse(), 10, 10, 1000)
