import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import javafx.application.Platform

class PhysicsManagerExample{
	def dev = DeviceManager.getSpecificDevice("hidDevice")
	MobileBase cat = DeviceManager.getSpecificDevice("MediumKat")
	DHParameterKinematics head = null
	DHParameterKinematics tail = null
	double[] imuDataValues = dev.simple.getImuData() // data kept up to date by device coms thread

	// Object variables
	double balenceAngle=0
     def groupA = cat.getLegs().subList(0, 2)
     def groupB = cat.getLegs().subList(2, 4)
     CSG frontFoot = new Cube(20).toCSG().setColor(javafx.scene.paint.Color.GREEN)
     CSG backFoot = new Cube(20).toCSG().setColor(javafx.scene.paint.Color.RED)
     CSG CoMwithHeadAndTail = new Cube(20).toCSG().setColor(javafx.scene.paint.Color.WHEAT)
     CSG CoMwithoutHeadOrTail = new Cube(20).toCSG().setColor(javafx.scene.paint.Color.LIGHTBLUE)
     CSG tiltLine = new Cube(200, 2, 2).toCSG().toXMin().setColor(javafx.scene.paint.Color.WHITE)
     TransformNR robotCoM = new TransformNR()

	boolean connected=false;
	double timeBase = 300
	long lastTimeTailCompletedSpin = 0
	long startTime=0;
	double maxOffset=40
	ArrayList<DHParameterKinematics> feetTouchingGround
	boolean poseUpdate=false
	double timeOfLaseSend=0
	Runnable event = {
		//println("physics event")
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
				//println("tilt="+tilt)
				double dt = System.currentTimeMillis() - lastTimeTailCompletedSpin
				double scaledTimeComponent = (dt / timeBase) * 2 * Math.PI
				if (tilt < 0) {
					scaledTimeComponent *= -1
				}
				SinComponent = Math.sin(scaledTimeComponent) * tilt * tailRotationGain
				CosComponent = Math.cos(scaledTimeComponent) * tilt * tailRotationGain
				//println("SinComponent="+SinComponent)
				//println("CosComponent="+CosComponent)
				//SinComponent = 0
				//CosComponent = 0

				if (dt >= timeBase) {
					lastTimeTailCompletedSpin = System.currentTimeMillis()
				}
			}

			List<TransformNR> downGroupTipsInWorldSpace = getDownGroupTipsInWorldSpace()
			def fTip = downGroupTipsInWorldSpace[0]
			def rTip = downGroupTipsInWorldSpace[1]
			double beta = Math.toDegrees(Math.atan2(fTip.getY()-rTip.getY(), fTip.getX()-rTip.getX()))
			TransformNR T_beta = new TransformNR(0, 0, 0, new RotationNR(0, beta, 0))
			TransformNR T_tilt = T_beta.inverse().times(
				new TransformNR(rTip.getX(), rTip.getY(), 0, new RotationNR()).inverse()
			).inverse()
			
			double xComp = 0.0
			double yComp = 0.0
			double zComp = 0.0
			double totalMass = 0.0
			for (int legIndex = 0; legIndex < cat.getLegs().size(); legIndex++) {
				def leg = cat.getLegs()[legIndex]
				for (int linkIndex = 0; linkIndex < leg.getChain().getLinks().size(); linkIndex++) {
					def CoM = linkCoM(leg, linkIndex)
					def mass = linkMass(leg, linkIndex)
					xComp += CoM.getX() * mass
					yComp += CoM.getY() * mass
					zComp += CoM.getZ() * mass
					totalMass += mass
				}
			}
			
			TransformNR CoMtail0 = linkCoM(tail, 0)
			double tail0Mass = linkMass(tail, 0)
			xComp += CoMtail0.getX() * tail0Mass
			yComp += CoMtail0.getY() * tail0Mass
			zComp += CoMtail0.getZ() * tail0Mass
			totalMass += tail0Mass

			TransformNR CoMhead0 = linkCoM(head, 0)
			double head0Mass = linkMass(head, 0)
			xComp += CoMhead0.getX() * head0Mass
			yComp += CoMhead0.getY() * head0Mass
			zComp += CoMhead0.getZ() * head0Mass
			totalMass += head0Mass

			TransformNR CoMbody = new TransformNR(2.15767635, -10, 115, new RotationNR())
			double bodyMass = 0.3856
			xComp += CoMbody.getX() * bodyMass
			yComp += CoMbody.getY() * bodyMass
			zComp += CoMbody.getZ() * bodyMass
			totalMass += bodyMass

			def tailYawLink = tail.getAbstractLink(1)
			TransformNR bestCoM = new TransformNR(1e+10, 1e+10, 1e+10, new RotationNR())
			for (double i = tailYawLink.getMinEngineeringUnits(); i < tailYawLink.getMaxEngineeringUnits(); i += 0.53) {
				double testXComp = xComp
				double testYComp = yComp
				double testZComp = zComp
				double testTotalMass = totalMass
				
				TransformNR CoMtail1 = linkCoM(tail, i, 1)
				double tail1Mass = linkMass(tail, 1)
				testXComp += CoMtail1.getX() * tail1Mass
				testYComp += CoMtail1.getY() * tail1Mass
				testZComp += CoMtail1.getZ() * tail1Mass
				testTotalMass += tail1Mass

				//TransformNR CoMhead1 = linkCoM(head, i, 1)
				TransformNR CoMhead1 = linkCoM(head, 1)
				double head1Mass = linkMass(head, 1)
				testXComp += CoMhead1.getX() * head1Mass
				testYComp += CoMhead1.getY() * head1Mass
				testZComp += CoMhead1.getZ() * head1Mass
				testTotalMass += head1Mass
				
				TransformNR T_CoMrobot = T_tilt.times(
					new TransformNR(
						testXComp / testTotalMass, testYComp / testTotalMass, testZComp / testTotalMass,
						new RotationNR()
					).inverse()
				)
				/*TransformNR T_CoMrobot = new TransformNR(
					testXComp / testTotalMass, testYComp / testTotalMass, testZComp / testTotalMass,
					new RotationNR()
				)*/

				//println(T_CoMrobot.getY())
				
				if (Math.abs(T_CoMrobot.getY()) < Math.abs(bestCoM.getY())) {
					//println("Saved new best CoM at tail angle " + i)
					//println("New CoM.y: " + T_CoMrobot.getY())
					balenceAngle = i
					bestCoM = T_CoMrobot
				}
			}
			//throw fhdskfskjfkjds

			robotCoM = bestCoM.copy()
			robotCoM.setZ(0)
			//println("CoM.y before tail correction: " + yComp / totalMass)
			//println("CoM.y after tail correction: " + bestCoM.getY())
			
			if(System.currentTimeMillis() > timeOfLaseSend + 20) {
				timeOfLaseSend=System.currentTimeMillis()
				// update the pose values for the robot
				try {
					Platform.runLater({
					TransformFactory.nrToAffine(downGroupTipsInWorldSpace[0], frontFoot.getManipulator())
					TransformFactory.nrToAffine(downGroupTipsInWorldSpace[1], backFoot.getManipulator())
					TransformFactory.nrToAffine(robotCoM, CoMwithHeadAndTail.getManipulator())
					TransformFactory.nrToAffine(
						new TransformNR(
							xComp / totalMass, yComp / totalMass, 0, new RotationNR()),
						CoMwithoutHeadOrTail.getManipulator())
					TransformFactory.nrToAffine(T_tilt, tiltLine.getManipulator())
				})
				}catch(Throwable ex) {
					ex.printStackTrace()
				}
			}
			if(head!=null && tail!=null){
				//println "Values:"+balenceAngle
				//boundSet(head,1,-balenceAngle)
				double[] vect =tail.getCurrentJointSpaceVector()
				vect[0]=bound(tail,0,-CosComponent*maxOffset)
				vect[1]=bound(tail,1,balenceAngle+(SinComponent*maxOffset))
				//tail.setDesiredJointSpaceVector(vect, 0)
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

		if (groupAIsDown) {
			return [groupA[1].getCurrentPoseTarget(), groupA[0].getCurrentPoseTarget()]
		} else {
			return [groupB[0].getCurrentPoseTarget(), groupB[1].getCurrentPoseTarget()]
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
		BowlerStudioController.addCsg(frontFoot)
		BowlerStudioController.addCsg(backFoot)
		BowlerStudioController.addCsg(CoMwithHeadAndTail)
		BowlerStudioController.addCsg(CoMwithoutHeadOrTail)
		BowlerStudioController.addCsg(tiltLine)
		return true
	}
	public void disconnect() {
		println "\r\nDisconnecting the PhysicsManager"
		dev.simple.removeEvent(1804,event);
		BowlerStudioController.clearCSG()
		BowlerStudioController.setCsg(MobileBaseCadManager.get(cat), null)
	}
}

return DeviceManager.getSpecificDevice("PhysicsManagerInstance", {
	return new 	PhysicsManagerExample()
})
