import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.common.Log

/**
 * Home all the legs immediately.
 *
 * @param base The base to home the legs of.
 */
void homeLegs(MobileBase base) {
    base.getLegs().each {
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
TransformNR solveForTipPositionInWorldSpace(
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
List<List<TransformNR>> computeInterpolation(
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
void followTransforms(
        List<DHParameterKinematics> legs,
        List<List<TransformNR>> points,
        long timeMs) {
    long timePerIteration = (long) (timeMs / (double) points.size())

    for (int i = 0; i < points.size(); i++) {
        List<TransformNR> tipPositions = points[i]

        for (int j = 0; j < legs.size(); j++) {
            def leg = legs[j]
            def tipTargetInWorldSpace = tipPositions[j]

            // Only move to the tip target if it is reachable or else the IK blows up
            if (leg.checkTaskSpaceTransform(tipTargetInWorldSpace)) {
                leg.setDesiredTaskSpaceTransform(tipTargetInWorldSpace.times(new TransformNR(10, 0, 0, new RotationNR())), 0)
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
void moveBaseWithLimbsPlanted(
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
List<TransformNR> createTrapezoidProfile(
	TransformNR baseDelta,
	double stepHeight) {
    TransformNR quarterBaseDelta = baseDelta.scale(1 / 4.0)
    TransformNR quarterBaseDeltaInverted = baseDelta.inverse().scale(1 / 4.0)
	return [
            baseDelta.scale(1 / 12.0),
            baseDelta.scale(1 / 12.0),
            baseDelta.scale(1 / 12.0),
            baseDelta.scale(1 / 4.0).times(new TransformNR(0, 0, stepHeight, new RotationNR())),
            baseDelta.inverse().scale(1 / 4.0),
            baseDelta.inverse().scale(1 / 4.0),
            baseDelta.inverse().scale(1 / 4.0),
            baseDelta.inverse().scale(1 / 4.0),
            baseDelta.scale(1 / 4.0).times(new TransformNR(0, 0, -stepHeight, new RotationNR())),
            baseDelta.scale(1 / 12.0),
            baseDelta.scale(1 / 12.0),
            baseDelta.scale(1 / 12.0)
    ]
}

/**
 * Creates a square-ish profile.
 * @param baseDelta The delta to apply to the base.
 * @param stepHeight The height of one step.
 * @return The profile.
 */
List<TransformNR> createSquareProfile(
	TransformNR baseDelta,
	double stepHeight) {
    TransformNR quarterBaseDelta = baseDelta.scale(1 / 4.0)
    TransformNR quarterBaseDeltaInverted = baseDelta.inverse().scale(1 / 4.0)
	return [
            quarterBaseDelta,
            quarterBaseDelta,
            quarterBaseDelta,
            // Keep the foot moving back when we pick it up
            quarterBaseDelta.times(new TransformNR(0, 0, stepHeight, new RotationNR())),
            quarterBaseDeltaInverted,
            quarterBaseDeltaInverted,
            quarterBaseDeltaInverted,
            quarterBaseDeltaInverted,
            // Put the foot straight down to the home position to avoid pushing backwards
            new TransformNR(0, 0, -stepHeight, new RotationNR())
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
List<TransformNR> createLimbTipMotionProfile(
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
                println("New tip:\n" + newTip + "\n")
                println("Body delta:\n" + profileBodyDelta + "\n")
                println("New body:\n" + globalFiducial.times(profileBodyDelta) + "\n")

                throw new UnsupportedOperationException(
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
List<List<List<TransformNR>>> computeInterpolatedGroupProfile(
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
void followInterpolatedGroupProfiles(
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

def createGroupAndProfile(
		group,
		int offset,
		TransformNR globalFiducial,
		TransformNR baseDelta,
		double stepHeight,
		int numberOfIncrements) {
	def profile = createLimbTipMotionProfile(globalFiducial, baseDelta, group, stepHeight)
	def interpolatedProfile = computeInterpolatedGroupProfile(
		group,
		profile,
		globalFiducial,
		numberOfIncrements,
		offset
	)
	return new Tuple(group, interpolatedProfile)
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
void walkBase(
        MobileBase base,
        TransformNR fiducialToGlobal,
        TransformNR baseDelta,
        double stepHeight,
        int numberOfIncrements,
        long timeMs) {
    def groupA = base.getLegs().subList(0, 2)
    def groupB = base.getLegs().subList(2, 4)

    // TODO: Split baseDelta if it is impossible
    def profileA = createLimbTipMotionProfile(fiducialToGlobal, baseDelta, groupA, stepHeight)
    def profileB = createLimbTipMotionProfile(fiducialToGlobal, baseDelta, groupB, stepHeight)

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
            6
    )

    followInterpolatedGroupProfiles(
            [groupA, groupB],
            [interpolatedProfileA, interpolatedProfileB],
            timeMs
    )
    /*def globalFiducial = base.getFiducialToGlobalTransform()
    def (groupA, profileA) = createGroupAndProfile([base.getLegs()[0]], 0, globalFiducial, baseDelta, stepHeight, numberOfIncrements)
    def (groupB, profileB) = createGroupAndProfile([base.getLegs()[1]], 0, globalFiducial, baseDelta, stepHeight, numberOfIncrements)
    def (groupC, profileC) = createGroupAndProfile([base.getLegs()[2]], 0, globalFiducial, baseDelta, stepHeight, numberOfIncrements)
    def (groupD, profileD) = createGroupAndProfile([base.getLegs()[3]], 0, globalFiducial, baseDelta, stepHeight, numberOfIncrements)
    followInterpolatedGroupProfiles(
    		[groupA, groupB, groupC, groupD],
    		[profileA, profileB, profileC, profileD],
    		timeMs
    )*/
}

Log.enableSystemPrint(true)

MobileBase base = DeviceManager.getSpecificDevice("MediumKat") as MobileBase
if (base == null) {
    throw new IllegalStateException("MediumKat device was null.");
}

homeLegs(base)
Thread.sleep(500)

TransformNR fiducialToGlobal = base.getFiducialToGlobalTransform()
println("Starting fiducialToGlobal:\n" + fiducialToGlobal + "\n")

TransformNR adjustRideHeight = new TransformNR(0, 0, 5, new RotationNR())
//moveBaseWithLimbsPlanted(base, fiducialToGlobal, adjustRideHeight, 100, 200L)
//fiducialToGlobal = fiducialToGlobal.times(adjustRideHeight)

double stepLength = 20
double stepHeight = 7
long timePerWalk = 250
for (int i = 0; i < 15; i++) {
	walkBase(base, fiducialToGlobal, new TransformNR(stepLength, 0, 0, new RotationNR(0, 0, 0)).inverse(), stepHeight, 10, timePerWalk)
}
