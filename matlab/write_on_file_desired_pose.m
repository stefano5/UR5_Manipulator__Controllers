path= 'C:\Users\stefa\Desktop\Robotica\Manipulator - non linear control\results_simulation/';



writematrix(out.refEE.Data(:,1), path + "xirefX.dat")
writematrix(out.refEE.Data(:,2), path + "xirefY.dat")
writematrix(out.refEE.Data(:,3), path + "xirefZ.dat")
writematrix(out.refEE.Data(:,4), path + "xirefR.dat")
writematrix(out.refEE.Data(:,5), path + "xirefP.dat")
writematrix(out.refEE.Data(:,6), path + "xirefYaw.dat")

writematrix(out.EEPose.Data(:,1), path + "xiX.dat")
writematrix(out.EEPose.Data(:,2), path + "xiY.dat")
writematrix(out.EEPose.Data(:,3), path + "xiZ.dat")
writematrix(out.EEPose.Data(:,4), path + "xiR.dat")
writematrix(out.EEPose.Data(:,5), path + "xiP.dat")
writematrix(out.EEPose.Data(:,6), path + "xiYaw.dat")

%write_on_file_desired_pose