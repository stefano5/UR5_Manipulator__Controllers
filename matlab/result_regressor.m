%%
thickness = 1.6;

%%
masses = [m1 m2 m3 m4 m5 m6];
%Inertias = [ur5.links(1,1).I, ur5.links(1,2).I, ur5.links(1,3).I, ur5.links(1,4).I, ur5.links(1,5).I, ur5.links(1,6).I];

%tensors_maessses
%% primo plot
figure % masses
for i = 1:6
    subplot(3,2, i);
    hold on
    plot(out.estim_masses.Time,out.estim_masses.Data(:,i),'k','LineWidth', thickness)    
    plot(out.estim_masses.Time,masses(i)*ones(size(out.estim_masses.Data(:,i))),'--r','LineWidth', thickness)    
    hold off
    xlabel('Time (s)');
    ylabel('kg');
    grid
    titl=join(['m_', string(i), '(t)']);
    title(titl)
    legend('estim','real')
end

%%

real_in = zeros(36,1);
for i = 1: 6
    real_in((i-1)*6+1) = ur5.links(1,i).I(1,1);
    real_in((i-1)*6+2) = ur5.links(1,i).I(1,2);
    real_in((i-1)*6+3) = ur5.links(1,i).I(1,3);
    real_in((i-1)*6+4) = ur5.links(1,i).I(2,2);
    real_in((i-1)*6+5) = ur5.links(1,i).I(2,3);
    real_in((i-1)*6+6) = ur5.links(1,i).I(3,3);
end

tensors_name=["_{xx}","_{xy}", "_{xz}", "_{yy}", "_{yz}", "_{zz}"];

for arm=1:6
    figure % q
    for i = 1:6
        subplot(3,2, i);
        hold on
        plot(out.estim_masses.Time,out.tensors_masses.Data(:,i + 6*(arm-1) ),'k','LineWidth', thickness)    
        plot(out.estim_masses.Time,real_in(i +(arm-1)*6)*ones(size(out.estim_masses.Data(:,i))),'--r','LineWidth', thickness)    
        hold off
        xlabel('Time (s)');
        ylabel('kg m^2');
        grid
        titl=join(["I", tensors_name(i), '(t)']);
        title(titl)
        legend('estim','real')
    end
end

