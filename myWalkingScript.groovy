void homeLegs(MobileBase base) {
	base.getLegs().each {
		it.setDesiredTaskSpaceTransform(it.calcHome(), 0)
	}
}

TransformNR solveForTipPositionInWorldSpace(MobileBase base, DHParameterKinematics leg, TransformNR baseDelta) {
	def T_tipGlobal = leg.getCurrentPoseTarget()
	def T_fiducialLimb = leg.getRobotToFiducialTransform()
	def T_globalFiducial = base.getFiducialToGlobalTransform()

	def tipInLimbSpace = leg.inverseOffset(T_tipGlobal)
	def newBodyPose = T_globalFiducial.times(baseDelta)
	return newBodyPose.times(T_fiducialLimb).times(tipInLimbSpace)
}

void moveBase(MobileBase base, TransformNR baseDelta, int numberOfIncrements) {
	for (int i = 0; i <= numberOfIncrements; i++) {
		double scale = i / numberOfIncrements

		base.getLegs().each { leg ->
			def tipTargetInWorldSpace = solveForTipPositionInWorldSpace(base, leg, baseDelta.scale(scale))
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

def T_deltaBody = new TransformNR(0, 0, 10, new RotationNR(0, 0, 0)).inverse()

homeLegs(base)
Thread.sleep(1000)
moveBase(base, T_deltaBody, 100)
Thread.sleep(1000)
moveBase(base, T_deltaBody.inverse(), 100)
