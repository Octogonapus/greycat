MobileBase base = DeviceManager.getSpecificDevice("MediumKat")
if (base == null) {
	throw new IllegalStateException("MediumKat device was null.");
}

def T_deltaBody = new TransformNR(0, 0, 0, new RotationNR(0, 20, 0)).inverse()

void homeLegs(MobileBase base) {
	base.getLegs().each {
		it.setDesiredTaskSpaceTransform(it.calcHome(), 0)
	}
}

void moveBase(MobileBase base, TransformNR baseDelta) {
	base.getLegs().each { leg ->
		def T_tipGlobal = leg.getCurrentPoseTarget()
		def T_fiducialLimb = leg.getRobotToFiducialTransform()
		def T_globalFiducial = base.getFiducialToGlobalTransform()
	
		def tipInLimbSpace = leg.inverseOffset(T_tipGlobal)
		def newBodyPose = T_globalFiducial.times(baseDelta)
		def newTipInWorldSpace = newBodyPose.times(T_fiducialLimb).times(tipInLimbSpace)
		
		if (leg.checkTaskSpaceTransform(newTipInWorldSpace)) {
			leg.setDesiredTaskSpaceTransform(newTipInWorldSpace, 0)
		}
	}
}

homeLegs(base)
moveBase(base, T_deltaBody)

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
