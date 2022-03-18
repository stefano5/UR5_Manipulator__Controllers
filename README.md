MATLAB dependencies are: the Robotics Toolbox created by P. Corke.

P.I. Corke, Robotics, Vision & Control: Fundamental Algorithms in MATLAB. Second edition. Springer, 2017. ISBN 978-3-319-54413-7.

To install this tool see: https://petercorke.com/toolboxes/robotics-toolbox/  (I suggest you the" Install from shared MATLAB Drive folder")



NB: if you don't install the Robotics Toolbox these matlab scripts won't work!





Different control tecniques were developed. See "Manipulator UR5 presentation.pptx".

Inside the 'matlab' folder there are three simulink files:
	Backstepping_and_CT.slx			<= it contains both the backstepping controller and the computed torque controller
	Adaptive_backstepping.slx		<= it contains the adaptive backstepping controller
	li_slotine.slx				<= it contains the li-slotine controller

To run one of these just open it and click on the run button and wait until the simulation ends (about 30 seconds). 
Then you will see the meaningful plots.



If you want to see the simulation video, run the file:

		manipulator_visualization_matlab.nb

then evaluate the notebook (enable the dynamics, if asked) and press "play".
