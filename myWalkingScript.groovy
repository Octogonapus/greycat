class InterpolationIterator {

	private Closure<TransformNR> computeNextTransform;

	public InterpolationIterator(Closure<TransformNR> computeNextTransform) {
		this.computeNextTransform = computeNextTransform
	}

	public TransformNR next() {
		return computeNextTransform()
	}
}

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
		legs,
		originalTipPositions,
		TransformNR globalFiducial,
		TransformNR baseDelta,
		int numberOfIncrements) {
	List<List<TransformNR>> points = new ArrayList<>(numberOfIncrements)
	
	for (int i = 0; i <= numberOfIncrements; i++) {
		double scale = i / (double) numberOfIncrements
		List<TransformNR> legPoints = new ArrayList<>(legs.size())
		
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

void followTransforms(legs, List<List<TransformNR>> points, int timeMs) {
	int timePerIteration = timeMs / (double) points.size()
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
		MobileBase base,
		TransformNR globalFiducial,
		TransformNR baseDelta,
		limbGroup,
		double stepHeight) {
	TransformNR quarterBaseDelta = baseDelta.scale(1/4.0)
	TransformNR quarterBaseDeltaInverted = baseDelta.inverse().scale(1/4.0)
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
		group,
		profile,
		TransformNR globalFiducial,
		int numberOfIncrements,
		int startIndex) {
	List<List<List<TransformNR>>> out = new ArrayList<>(profile.size() + 1)
	
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

void followInterpolatedGroupProfile(
	group,
	List<List<List<TransformNR>>> interpolatedProfile,
	int timeMs) {
	int timeMsPerProfileStep = timeMs / (double) (interpolatedProfile.size() - 1)

	followTransforms(
		group,
		interpolatedProfile[0],
		0
	)

	for (int i = 0; i < interpolatedProfile.size() - 1; i++) {
		followTransforms(
			group,
			interpolatedProfile[i + 1],
			timeMsPerProfileStep
		)
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
	def profileA = createLimbTipMotionProfile(base, globalFiducial, baseDelta, groupA, stepHeight)
	def profileB = createLimbTipMotionProfile(base, globalFiducial, baseDelta, groupB, stepHeight)

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

	followInterpolatedGroupProfile(
		groupA,
		interpolatedProfileA,
		timeMs
	)
	followInterpolatedGroupProfile(
		groupB,
		interpolatedProfileB,
		timeMs
	)
}

Log.enableSystemPrint(true)

MobileBase base = DeviceManager.getSpecificDevice("MediumKat")
if (base == null) {
	throw new IllegalStateException("MediumKat device was null.");
}

def T_pushup = new TransformNR(0, 0, 10, new RotationNR(0, 0, 0)).inverse()
def T_twist = new TransformNR(0, 0, 0, new RotationNR(0, 5, 0)).inverse()

homeLegs(base)
Thread.sleep(500)

walkBase(base, new TransformNR(30, 0, 0, new RotationNR(0, 0, 0)).inverse(), 10, 10, 5000)
