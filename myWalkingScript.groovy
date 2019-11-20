void homeLegs(MobileBase base) {
	base.getLegs().each {
		it.setDesiredTaskSpaceTransform(it.calcHome(), 0)
	}
}

TransformNR solveForTipPositionInWorldSpace(DHParameterKinematics leg, TransformNR originalTipInLimbSpace, TransformNR newBodyPose) {
	def T_tipGlobal = originalTipInLimbSpace
	def T_fiducialLimb = leg.getRobotToFiducialTransform()
	def tipInLimbSpace = leg.inverseOffset(T_tipGlobal)
	return newBodyPose.times(T_fiducialLimb).times(tipInLimbSpace)
}

void interpolateAndRun(legs, originalTipPositions, TransformNR globalFiducial, TransformNR baseDelta, int numberOfIncrements) {
	for (int i = 0; i <= numberOfIncrements; i++) {
		double scale = i / (double) numberOfIncrements

		for (int j = 0; j < legs.size(); j++) {
			def leg = legs[j]
			
			def tipTargetInWorldSpace = solveForTipPositionInWorldSpace(
				leg,
				originalTipPositions[j],
				globalFiducial.times(baseDelta.scale(scale))
			)

			if (leg.checkTaskSpaceTransform(tipTargetInWorldSpace)) {
				leg.setDesiredTaskSpaceTransform(tipTargetInWorldSpace, 0)
			}
		}

		Thread.sleep(10)
	}
}

void moveBaseWithLimbsPlanted(MobileBase base, TransformNR baseDelta, int numberOfIncrements) {
	def originalTipPositionsInLimbSpace = base.getLegs().collect { leg ->
		leg.getCurrentPoseTarget()
	}

	interpolateAndRun(base.getLegs(), originalTipPositionsInLimbSpace, base.getFiducialToGlobalTransform(), baseDelta, numberOfIncrements)
}

List<TransformNR> createLimbTipMotionProfile(MobileBase base, TransformNR globalFiducial, TransformNR baseDelta, limbGroup, double stepHeight) {
	def profile = [
		baseDelta.scale(1/4.0),
		baseDelta.scale(1/4.0).times(new TransformNR(0, 0, stepHeight, new RotationNR())),
		baseDelta.inverse().scale(1/4.0),
		baseDelta.inverse().scale(1/4.0),
		baseDelta.inverse().scale(1/4.0),
		baseDelta.inverse().scale(1/4.0),
		baseDelta.scale(1/4.0).times(new TransformNR(0, 0, -stepHeight, new RotationNR())),
		baseDelta.scale(1/4.0)
	]

	def startingTipPositions = limbGroup.collect { leg ->
		leg.calcHome()
	}

	for (int i = 0; i < profile.size(); i++) {
		def profileBodyDelta = profile[i]
		def newBase = globalFiducial.times(profileBodyDelta)
		
		for (int j = 0; j < limbGroup.size(); j++) {
			def leg = limbGroup[j]
			def newTip = solveForTipPositionInWorldSpace(leg, startingTipPositions[j], newBase)

			if (!leg.checkTaskSpaceTransform(newTip)) {
				println("New tip:\n" + newTip + "\n")
				println("Body delta:\n" + profileBodyDelta + "\n")
				println("New body:\n" + globalFiducial.times(profileBodyDelta) + "\n")
				throw new UnsupportedOperationException("Unreachable: profile index " + i + ", leg index " + j)
			}
			
			startingTipPositions[j] = newTip
		}
	}

	return profile
}

void followGroupProfile(group, profile, TransformNR globalFiducial, int startIndex) {
	def startingTipPositions = group.collect { leg ->
		leg.calcHome()
	}

	for (int i = 0; i < startIndex; i++) {
		def bodyDelta = profile[i]
		def newBody = globalFiducial.times(bodyDelta)
		
		for (int j = 0; j < group.size(); j++) {
			startingTipPositions[j] = solveForTipPositionInWorldSpace(group[j], startingTipPositions[j], newBody)
		}
	}

	interpolateAndRun(group, startingTipPositions, globalFiducial, new TransformNR(0, 0, 0, new RotationNR()), 1)
	
	for (int i = 0; i < profile.size(); i++) {
		def adjustedIndex = i + startIndex
		if (adjustedIndex >= profile.size()) {
			adjustedIndex -= profile.size()
		}
		
		def bodyDelta = profile[adjustedIndex]
		def newBody = globalFiducial.times(bodyDelta)

		interpolateAndRun(group, startingTipPositions, globalFiducial, bodyDelta, 50)
		
		for (int j = 0; j < group.size(); j++) {
			startingTipPositions[j] = solveForTipPositionInWorldSpace(group[j], startingTipPositions[j], newBody)
		}
	}
}

void walkBase(MobileBase base, TransformNR baseDelta, double stepHeight) {
	def groupA = base.getLegs().subList(0, 2)
	def groupB = base.getLegs().subList(2, 4)

	// TODO: Split baseDelta if it is impossible
	def globalFiducial = base.getFiducialToGlobalTransform()
	def profileA = createLimbTipMotionProfile(base, globalFiducial, baseDelta, groupA, stepHeight)
	def profileB = createLimbTipMotionProfile(base, globalFiducial, baseDelta, groupB, stepHeight)

	Thread.start {
		followGroupProfile(groupA, profileA, globalFiducial, 0)
	}
	Thread.start {
		followGroupProfile(groupB, profileB, globalFiducial, 3)
	}
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

walkBase(base, new TransformNR(20, 0, 0, new RotationNR(0, 0, 0)).inverse(), 10)
