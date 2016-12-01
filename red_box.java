package application;


import java.io.IOException;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.executionModel.IFiredConditionInfo;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.CartPlane;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.SPL;
import com.kuka.roboticsAPI.motionModel.Spline;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianSineImpedanceControlMode;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class red_box extends RoboticsAPIApplication {
	private Controller kuka_Sunrise_Cabinet_1;
	private LBR lbr_iiwa_7_R800_1;
	private Tool gripper;
	private RobotLogger logger;
	public void initialize() {
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		lbr_iiwa_7_R800_1 = (LBR) getDevice(kuka_Sunrise_Cabinet_1,
				"LBR_iiwa_7_R800_1");
		logger = new RobotLogger("//128.8.140.151/data/Trial36.txt", 10.0);
	}
	
	
	public int FindClosestFrame(Frame current, Frame[] FrameArray){ //returns index of closest frame
		double minDistance = FrameArray[0].distanceTo(current);
		int index = 0;
		for (int i = 1;  i <FrameArray.length; i++){
			if (FrameArray[i].distanceTo(current) < minDistance)
			{
				minDistance = FrameArray[i].distanceTo(current);
				index = i;
			}
		}
		return index;
	}

	public void run() {
		gripper=getApplicationData().createFromTemplate("Robotiq");
		gripper.attachTo(lbr_iiwa_7_R800_1.getFlange());
		logger.addCartesianForce(gripper.getFrame("/Gripper_TCP"), null);
		logger.addCurrentCartesianPositionXYZ(gripper.getFrame("/Gripper_TCP"), getApplicationData().getFrame("/Base"));
		logger.enable();
		double x[] = {392.712000,383.497000,374.266000,365.085000,388.121000,378.850000,369.577000,360.391000,392.707000,383.505000,374.123000,364.858000,388.006000,378.703000,369.399000,360.122000,392.648000,383.358000,373.980000,364.638000,387.965000,378.590000,369.201000,359.877000,392.488000,383.195000,373.814000,364.417000,387.756000,378.359000,368.998000,359.612000}; 
		double y[] = {23.179900,23.384500,23.561200,23.722500,18.707400,18.910700,19.087200,19.259900,13.972200,14.186300,14.381900,14.583100,9.423230,9.629200,9.828040,10.056200,4.635680,4.818800,5.036910,5.285440,0.012723,0.214515,0.446414,0.724121,-4.738890,-4.600580,-4.426710,-4.124490,-9.368260,-9.124820,-8.955650,-8.761440}; 
		double z[] = {424.067000,424.367000,426.832000,430.382000,419.453000,420.096000,424.263000,428.667000,417.510000,413.305000,419.570000,422.003000,415.896000,415.974000,413.900000,420.872000,412.698000,410.337000,411.750000,416.720000,409.180000,409.022000,410.957000,416.210000,411.282000,408.116000,406.660000,411.701000,409.408000,411.084000,409.838000,409.566000}; 

	
		double A,B,C;
		double speed =0.1;
		double normal_force = 15;
		
		int N=x.length;
		
		A=Math.PI;
		B=0;
		C=-Math.PI;
		
		CartesianSineImpedanceControlMode mode=   CartesianSineImpedanceControlMode.createSinePattern(CartDOF.Y, 2.0,10.0, 500.0);
				
		
		mode.parametrize(CartDOF.Z).setBias(10);
		
		Frame safety = new Frame(344.03,114.60,636.70,A,B,C); //Gripper_TCP
			Frame[] F = new Frame [N];
		SPL[] spls = new SPL[N];
		
		for(int i=0;i<N;i++)
		{
			F[i]=new Frame(x[i],y[i],z[i],A,B,C);
		}
		
		
		
		for(int i=0;i<N;i++)
		{
			F[i].setX(x[i]);
			F[i].setY(y[i]);
			F[i].setZ(z[i]+30);
			
			F[i].setAlphaRad(A);
			F[i].setBetaRad(B);
			F[i].setGammaRad(C);
			spls[i] = spl(F[i]);
		}	
		Frame fExceeded = null; //Frame where thing is hit
		gripper.getFrame("/Gripper_TCP").move(ptp(F[0]).setJointVelocityRel(speed).setMode(mode));
		Spline Spline1 = new Spline(spls);
		ForceCondition marker_normal_force=ForceCondition.createNormalForceCondition(gripper.getFrame("/Gripper_TCP"),CoordinateAxis.Z, normal_force);
		logger.startRecording();
		IMotionContainer motionCmd=gripper.getFrame("/Gripper_TCP").move(Spline1.setJointVelocityRel(speed).setMode(mode).breakWhen(marker_normal_force));
		IFiredConditionInfo firedInfo = motionCmd.getFiredBreakConditionInfo();
		if(firedInfo!=null)
				{
					getLogger().info("I am hit!");
					fExceeded = lbr_iiwa_7_R800_1.getCurrentCartesianPosition(gripper.getFrame("/Gripper_TCP"));
					gripper.getFrame("/Gripper_TCP").move(ptp(safety).setJointVelocityRel(speed));
					getLogger().info("Safe!");
				
		
					//wait a bit
		
					//determine closest index
					int index = FindClosestFrame(fExceeded, F);
					getLogger().info("Index: " + index);
					SPL[] newPath = new SPL[N-index];
					for (int i = 0; i < newPath.length; i++){
						newPath[i] = spl(F[i+index]);
					}
					Spline Spline2 = new Spline(newPath);
					gripper.getFrame("/Gripper_TCP").move(Spline2.setJointVelocityRel(speed).setMode(mode));
				}
		logger.stopRecording();
		try {
			logger.download();
				}
		catch (IOException err) {
				getLogger().error("File copy failed: " + err.getMessage());
		}
		gripper.getFrame("/Gripper_TCP").move(ptp(safety).setJointVelocityRel(speed)); //to make picture doesn't get messed up
		getLogger().info("points: "+ N);
			
	}

	/**
	 * Auto-generated method stub. Do not modify the contents of this method.
	 */
	public static void main(String[] args) {
		red_box app = new red_box();
		app.runApplication();
	}
}
