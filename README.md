# Dependencies

MATLAB dependencies are: the Robotics Toolbox created by P. Corke.

P.I. Corke, Robotics, Vision & Control: Fundamental Algorithms in MATLAB. Second edition. Springer, 2017. ISBN 978-3-319-54413-7.

To install this tool see: https://petercorke.com/toolboxes/robotics-toolbox/  (I suggest you to use the" Install from shared MATLAB Drive folder")


NB: if you don't install the Robotics Toolbox these matlab scripts won't work!


# Explanation

UR5 is a manipualtor with 6 degree of freedom (6DOF). 
These scripts were developed by Mariangela Menolotto and Stefano Maugeri as a project of the course "Robotics" at Università di Pisa. There they call these kind of projects "Tavola 1".

Different control tecniques were developed. For the theoretical part and more videos see:

	Manipulator UR5 presentation.pptx


# Simulink
Inside the 'matlab' folder there are three simulink files:
 - Backstepping_and_CT.slx			<= it contains both the backstepping controller and the computed torque controller
 - Adaptive_backstepping.slx			<= it contains the adaptive backstepping controller
 - li_slotine.slx					<= it contains the (also adaptive) li-slotine controller

To run one of these, just open it and click on the run button and wait until the simulation ends (about 30 seconds). 
Then you will see the meaningful plots.


# Mathematica
If you want to see the simulation video, run the Mathematica file (you need Mathematica):

		manipulator_visualization_matlab.nb

then evaluate the notebook (enable the dynamics, if asked) and press "play".

# Results
https://user-images.githubusercontent.com/40228829/158996613-95347114-3ea6-4a00-8a35-1c7bb0adadca.mp4

