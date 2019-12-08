import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

class PhysicsManagerExample{
	def dev = DeviceManager.getSpecificDevice("hidDevice")
	MobileBase cat = DeviceManager.getSpecificDevice("MediumKat")
	DHParameterKinematics head = null
	DHParameterKinematics tail = null
	double[] imuDataValues = dev.simple.getImuData() // data kept up to date by device coms thread

	// Object variables
	double balenceAngle=0

	boolean connected=false;
	double timeBase = 300
	long startTime=0;
	double maxOffset=40
	ArrayList<DHParameterKinematics> feetTouchingGround
	boolean poseUpdate=false
	Runnable event = {
		println("physics event")
		try {
			if(head==null || tail==null){
				for(DHParameterKinematics l:cat.getAllDHChains()) {
					if(l.getScriptingName().contains("Head")) {
						head=l
					}
					if(l.getScriptingName().contains("Tail")) {
						tail=l
					}
				}
			}
			double tilt = imuDataValues[10]
			double SinComponent=0
			double CosComponent=0
			double tailRotationGain = 1
			
			if(Math.abs(tilt)>3){
				// compute the values for the tail here
				double dt = System.currentTimeMillis() - timeOfLaseSend
				SinComponent = Math.sin(dt * 2 * Math.PI) * tilt * tailRotationGain
				CosComponent = Math.cos(dt * 2 * Math.PI) * tilt * tailRotationGain
				println("SinComponent="+SinComponent)
				println("CosComponent="+CosComponent)
			}
			if((System.currentTimeMillis()>timeOfLaseSend+20) &&poseUpdate ) {
				timeOfLaseSend=System.currentTimeMillis()
				// update the pose values for the robot
			}
			if(head!=null && tail!=null){
				//println "Values:"+balenceAngle
				boundSet(head,1,-balenceAngle)
				double[] vect =tail.getCurrentJointSpaceVector()
				vect[0]=bound(tail,0,-CosComponent*maxOffset)
				vect[1]=bound(tail,1,balenceAngle+(SinComponent*maxOffset))
				tail.setDesiredJointSpaceVector(vect, 0)
			}
		}catch(Exception ex) {
			ex.printStackTrace()
			disconnect() 
		}
		
	}
	
	private TransformNR linkCoM(DHParameterKinematics limb, double balenceLinkAngle ,int linkIndex) {
		//int linkIndex=1
		for(int i=0;i<5;i++) {
			try {
				double [] vectortail = limb.getCurrentJointSpaceVector();
				vectortail[linkIndex]=balenceLinkAngle;
				return limb.getChain().getChain(vectortail).get(linkIndex).
						times(limb.getLinkConfiguration(linkIndex).getCenterOfMassFromCentroid())
			}catch (Exception e) {
				Thread.sleep(0,20);
			}
		}
	}
	private TransformNR linkCoM(DHParameterKinematics limb ,int linkIndex) {
		return linkCoM(limb,limb.getCurrentJointSpaceVector()[linkIndex],linkIndex)
	}
	private double linkMass(DHParameterKinematics limb) {
		return limb.getLinkConfiguration(1).getMassKg()
	}
	void boundSet(DHParameterKinematics d, int index,double value) {
		value=bound(d,index,value)
		double[] vect =d.getCurrentJointSpaceVector()
		vect[index]=value
		d.setDesiredJointSpaceVector(vect, 0)
	}
	double bound(DHParameterKinematics d, int index,double value) {
		def l1 = d.getAbstractLink(index)
		if(value>l1.getMaxEngineeringUnits()){
			value=l1.getMaxEngineeringUnits();
		}
		if(value<l1.getMinEngineeringUnits()){
			value=l1.getMinEngineeringUnits();
		}
		return value
	}
	public updatePose(MobileBase cat ,ArrayList<DHParameterKinematics> feetTouchingGround) {
		this.cat=cat
		this.feetTouchingGround=feetTouchingGround
	     poseUpdate=true
	}
	public boolean connect() {
		if(connected) {
			println("PhysicsManager already connected")
			return
		}
		println "PhysicsManager Connecting "
		connected=true
		dev.simple.addEvent(1804, event);
		return true
	}
	public void disconnect() {
		println "\r\nDisconnecting the PhysicsManager"
		dev.simple.removeEvent(1804,event);
	
	}
}

return DeviceManager.getSpecificDevice("PhysicsManagerInstance", {
	return new 	PhysicsManagerExample()
})
