MobileBase base = DeviceManager.getSpecificDevice("MediumKat")
if (base == null) {
	throw new IllegalStateException("MediumKat device was null.");
}

double distanceToWalk = 500
double timeInSeconds = 5
double numberOfIncrements = 100
double scale = 1 / numberOfIncrements

def displacement = new TransformNR(distanceToWalk, 0, 0, new RotationNR())

for (int i = 0; i < numberOfIncrements; i++) {
	base.DriveArc(displacement.scale(scale), timeInSeconds * scale)
	Thread.sleep((long)(1000 * timeInSeconds * scale))
}
