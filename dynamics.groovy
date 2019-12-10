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
     def groupA = base.getLegs().subList(0, 2)
     def groupB = base.getLegs().subList(2, 4)

	boolean connected=false;
	double timeBase = 300
	long lastTimeTailCompletedSpin = 0
	long startTime=0;
	double maxOffset=40
	ArrayList<DHParameterKinematics> feetTouchingGround
	boolean poseUpdate=false
	double timeOfLaseSend=0
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
			double tailRotationGain = 1.0/90.0
			
			if(Math.abs(tilt)>3){
				// compute the values for the tail here
				println("tilt="+tilt)
				double dt = System.currentTimeMillis() - lastTimeTailCompletedSpin
				double scaledTimeComponent = (dt / timeBase) * 2 * Math.PI
				if (tilt > 0) {
					scaledTimeComponent *= -1
				}
				SinComponent = Math.sin(scaledTimeComponent) * tilt * tailRotationGain
				CosComponent = Math.cos(scaledTimeComponent) * tilt * tailRotationGain
				println("SinComponent="+SinComponent)
				println("CosComponent="+CosComponent)

				List<TransformNR> downGroupTipsInWorldSpace = getDownGroupTipsInWorldSpace()
				double beta = Math.atan2(
					downGroupTipsInWorldSpace[1].getY() - downGroupTipsInWorldSpace[0].getY(),
					downGroupTipsInWorldSpace[1].getX() - downGroupTipsInWorldSpace[0].getX()
				)
				TransformNR T_beta = new TransformNR(0, 0, 0, new RotationNR(0, beta, 0))
				TransformNR T_tilt = T_beta.inverse().times(new TransformNR(downGroupTipsInWorldSpace[0].getX(), downGroupTipsInWorldSpace[0].getY(), 0, new RotationNR()))

				double xComp = 0.0
				double yComp = 0.0
				double zComp = 0.0
				double totalMass = 0.0
				for (int legIndex = 0; legIndex < cat.getLegs().size(); legIndex++) {
					def leg = cat.getLegs()[legIndex]
					for (int linkIndex = 0; linkIndex < leg.getChain().size(); linkIndex++) {
						def CoM = linkCoM(leg, linkIndex)
						def mass = linkMass(leg, linkIndex)
						xComp += CoM.getX() * mass
						yComp += CoM.getY() * mass
						zComp += CoM.getZ() * mass
						totalMass += mass
					}
				}
				
				TransformNR CoMtail0 = linkCoM(tail, 0)
				xComp += CoMtail0.getX()
				yComp += CoMtail0.getY()
				zComp += CoMtail0.getZ()
				totalMass += linkMass(tail, 0)

				TransformNR CoMhead0 = linkCoM(head, 0)
				xComp += CoMhead0.getX()
				yComp += CoMhead0.getY()
				zComp += CoMhead0.getZ()
				totalMass += linkMass(head, 0)
				
				TransformNR T_CoMlegs = new TransformNR(xComp / totalMass, yComp / totalMass, zComp / totalMass, new RotationNR())

				def tailYawLink = d.getAbstractLink(1)
				TransformNR bestCoM = new TransformNR(1e+10, 1e+10, 1e+10, new RotationNR())
				for (int i = tailYawLink.getMinEngineeringUnits(); i < tailYawLink.getMaxEngineeringUnits(); i++) {
					TransformNR T_tail = linkCoM(tail, i, 1)
					TransformNR T_CoMrobot = T_tilt.times(T_CoMlegs).times(T_tail)
					if (Math.abs(T_CoMrobot.getY()) < Math.abs(bestCoM.getY())) {
						balenceAngle = i
						bestCoM = T_CoMrobot
					}
				}

				if (dt >= timeBase) {
					lastTimeTailCompletedSpin = System.currentTimeMillis()
				}
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

	private List<TransformNR> getDownGroupTipsInWorldSpace() {
		boolean groupAIsDown = true
		for (int i = 0; i < groupA.size(); i++) {
			for (int j = 0; j < groupB.size(); j++) {
				if (groupA[i].getCurrentPoseTarget().getZ() > groupB[j].getCurrentPoseTarget().getZ()) {
					groupAIsDown = false
					break
				}
			}
		}

		def downGroup = groupA
		if (!groupAIsDown) {
			downGroup = groupB
		}
		
		List<TransformNR> out = new ArrayList<TransformNR>()
		for (int i = 0; i < downGroup.size(); i++) {
			out.add(downGroup[i].getCurrentPoseTarget())
		}
		return out
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
	private double linkMass(DHParameterKinematics limb, int linkIndex) {
		return limb.getLinkConfiguration(linkIndex).getMassKg()
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
