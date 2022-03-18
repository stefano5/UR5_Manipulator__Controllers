path= 'C:\Users\stefa\Desktop\Robotica\Manipulator - non linear control\results_simulation/';

writematrix(out.q.Data(:,1), path + "q1.dat")
writematrix(out.q.Data(:,2), path + "q2.dat")
writematrix(out.q.Data(:,3), path + "q3.dat")
writematrix(out.q.Data(:,4), path + "q4.dat")
writematrix(out.q.Data(:,5), path + "q5.dat")
writematrix(out.q.Data(:,6), path + "q6.dat")


writematrix(out.EEPose.Data(:,1), path + "xirefX.dat")
writematrix(out.EEPose.Data(:,2), path + "xirefY.dat")
writematrix(out.EEPose.Data(:,3), path + "xirefZ.dat")
writematrix(out.EEPose.Data(:,4), path + "xirefR.dat")
writematrix(out.EEPose.Data(:,5), path + "xirefP.dat")
writematrix(out.EEPose.Data(:,6), path + "xirefY.dat")

write_on_file_desired_pose
