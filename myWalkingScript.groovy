MobileBase base = DeviceManager.getSpecificDevice("MediumKat")
if (base == null) {
	throw new IllegalStateException("MediumKat device was null.");
}

def T_deltaBody = new TransformNR(0, 0, 10, new RotationNR(0, 0, 0)).inverse()

void homeLegs(MobileBase base) {
	base.getLegs().each {
		it.setDesiredTaskSpaceTransform(it.calcHome(), 0)
	}
}

void moveBase(MobileBase base, TransformNR baseDelta, int numberOfIncrements) {
	def tipsInWorldSpace = base.getLegs().collect { leg ->
		def T_tipGlobal = leg.getCurrentPoseTarget()
		def T_fiducialLimb = leg.getRobotToFiducialTransform()
		def T_globalFiducial = base.getFiducialToGlobalTransform()
	
		def tipInLimbSpace = leg.inverseOffset(T_tipGlobal)
		def newBodyPose = T_globalFiducial.times(baseDelta)
		[leg, newBodyPose.times(T_fiducialLimb).times(tipInLimbSpace)]
	}

	for (int i = 0; i <= numberOfIncrements; i++) {
		double scale = i / numberOfIncrements
		
		tipsInWorldSpace.each { legAndTip ->
			def leg = legAndTip[0]
			def tipTargetInWorldSpace = legAndTip[1].scale(scale)
			if (leg.checkTaskSpaceTransform(tipTargetInWorldSpace)) {
				leg.setDesiredTaskSpaceTransform(tipTargetInWorldSpace, 0)
			}
		}
		
		Thread.sleep(10)
	}
}

//homeLegs(base)
moveBase(base, T_deltaBody, 1)
Thread.sleep(500)
moveBase(base, T_deltaBody.inverse(), 1)

/*DHParameterKinematics leg = base.getLegs()[0]
leg.setDesiredTaskSpaceTransform(leg.calcHome(), 0)

def T_deltaBody = new TransformNR(0, 0, 0, new RotationNR(0, 0, 0)).inverse()
def T_tipGlobal = leg.getCurrentPoseTarget()
def T_fiducialLimb = leg.getRobotToFiducialTransform()
def T_globalFiducial = base.getFiducialToGlobalTransform()

def tipInLimbSpace = leg.inverseOffset(T_tipGlobal)
def newBodyPose = T_globalFiducial.times(T_deltaBody)
def newTipInWorldSpace = newBodyPose.times(T_fiducialLimb).times(tipInLimbSpace)

if (leg.checkTaskSpaceTransform(newTipInWorldSpace)) {
	leg.setDesiredTaskSpaceTransform(newTipInWorldSpace, 0)
}*/

/*double distanceToWalk = 500
double timeInSeconds = 5
double numberOfIncrements = 100
double scale = 1 / numberOfIncrements

def displacement = new TransformNR(distanceToWalk, 0, 0, new RotationNR())

for (int i = 0; i < numberOfIncrements; i++) {
	base.DriveArc(displacement.scale(scale), timeInSeconds * scale)
	Thread.sleep((long)(1000 * timeInSeconds * scale))
}*/
