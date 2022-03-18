classdef UR5 < SerialLink
 
	properties
	end
 
	methods
		function ro = UR5()
			objdir = which('UR5');
			idx = find(objdir == filesep,2,'last');
			objdir = objdir(1:idx(1));
			 
			tmp = load(fullfile(objdir,'@UR5','matUR5.mat'));
			 
			ro = ro@SerialLink(tmp.sr);
			 
			 
		end
	end
	 
end
