void homeLegs(MobileBase base) {
	base.getLegs().each {
		it.setDesiredTaskSpaceTransform(it.calcHome(), 0)
	}
}

TransformNR solveForTipPositionInWorldSpace(
	MobileBase base,
	DHParameterKinematics leg,
	TransformNR originalTipInLimbSpace,
	TransformNR baseDelta) {
	def T_tipGlobal = originalTipInLimbSpace
	def T_fiducialLimb = leg.getRobotToFiducialTransform()
	def T_globalFiducial = base.getFiducialToGlobalTransform()

	def tipInLimbSpace = leg.inverseOffset(T_tipGlobal)
	def newBodyPose = T_globalFiducial.times(baseDelta)
	return newBodyPose.times(T_fiducialLimb).times(tipInLimbSpace)
}

void moveBaseWithLimbsPlanted(MobileBase base, TransformNR baseDelta, int numberOfIncrements) {
	def originalTipPositionsInLimbSpace = base.getLegs().collect { leg ->
		leg.getCurrentPoseTarget()
	}
	
	for (int i = 0; i <= numberOfIncrements; i++) {
		double scale = i / numberOfIncrements

		base.getLegs().eachWithIndex { leg, legIndex ->
			def tipTargetInWorldSpace = solveForTipPositionInWorldSpace(
				base,
				leg,
				originalTipPositionsInLimbSpace[legIndex],
				baseDelta.scale(scale)
			)
			
			if (leg.checkTaskSpaceTransform(tipTargetInWorldSpace)) {
				leg.setDesiredTaskSpaceTransform(tipTargetInWorldSpace, 0)
			}
		}
		
		Thread.sleep(10)
	}
}

MobileBase base = DeviceManager.getSpecificDevice("MediumKat")
if (base == null) {
	throw new IllegalStateException("MediumKat device was null.");
}

def T_pushup = new TransformNR(0, 0, 10, new RotationNR(0, 0, 0)).inverse()
def T_twist = new TransformNR(0, 0, 0, new RotationNR(0, 5, 0)).inverse()

homeLegs(base)
Thread.sleep(500)

moveBaseWithLimbsPlanted(base, T_pushup, 100)
Thread.sleep(500)
moveBaseWithLimbsPlanted(base, T_pushup.inverse(), 100)

Thread.sleep(500)

moveBaseWithLimbsPlanted(base, T_twist, 100)
Thread.sleep(500)
moveBaseWithLimbsPlanted(base, T_twist.inverse(), 100)
moveBaseWithLimbsPlanted(base, T_twist.inverse(), 100)
Thread.sleep(500)
moveBaseWithLimbsPlanted(base, T_twist, 100)

homeLegs(base)
Thread.sleep(500)
