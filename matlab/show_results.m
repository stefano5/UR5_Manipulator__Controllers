close all

%%

thickness = 1.8;
toDeg = 180/pi;

%fprintf('rank: %d\n', min(out.jacobianRank.Data))

%% primo plot
figure % q
for i = 1:6
    subplot(3,2, i);
    plot(out.q.Time,out.q.Data(:,i)*toDeg,'k','LineWidth', thickness)    
    xlabel('Time (s)');
    ylabel('deg');
    grid
    titl=join(['q_', num2str(i), '(t)']);
    title(titl)
end

%% secondo plot
figure % qdot
for i = 1:6
    subplot(3,2, i);
    plot(out.q.Time,out.qdot.Data(:,i)*toDeg,'k','LineWidth', thickness)
    xlabel('Time (s)');
    ylabel('deg/s');
    grid
    titl=join(['q_', num2str(i), ' dot(t)']);
    title(titl)
end

%% terzo plot
figure % end effector coordinates

titles=["x", "y", "z", "roll", "pitch", "yaw"];

for i = 1:6
    subplot(2,3, i);
    hold on
    if i< 4
        plot(out.EEPose.Time,out.EEPose.Data(:,i),'-k','LineWidth', thickness)
        plot(out.refEE.Time,out.refEE.Data(:,i),'--r','LineWidth', thickness)    
        ylabel('m');
    else
        plot(out.EEPose.Time,out.EEPose.Data(:,i)*toDeg,'-k','LineWidth', thickness)
        plot(out.refEE.Time,out.refEE.Data(:,i)*toDeg,'--r','LineWidth', thickness)    
        ylabel('deg');
    end
    hold off
    xlabel('Time (s)');
    legend('value','ref')
    grid
    % titl=join(['q_', string(i), '(t)']);
    title(titles(i))
end

%clear thickness

%% errori xi
figure
for i = 1:6
    subplot(3,2, i);
    plot(out.error.Time,out.error.Data(:,i)*toDeg,'k','LineWidth', thickness)    
    xlabel('Time (s)');
    ylabel('deg');
    grid
    
    titl=join([titles(i), " error"]);
    title(titl)
end
%%

%% torque
figure
for i = 1:6
    subplot(3,2, i);
    hold on
    plot(out.torque.Time,out.torque.Data(:,i),'k','LineWidth', thickness)    
    if i<4
        plot(out.torque.Time, 150 * ones(size(out.torque.Time,1),1),'--r','LineWidth', thickness)
        plot(out.torque.Time, -150 * ones(size(out.torque.Time,1),1),'--r','LineWidth', thickness)
    else
        plot(out.torque.Time, 28 * ones(size(out.torque.Time,1),1),'--r','LineWidth', thickness)
        plot(out.torque.Time, -28 * ones(size(out.torque.Time,1),1),'--r','LineWidth', thickness)
    end
    legend("Torque","Upper limit","Lower limit")
    hold off
    xlabel('Time (s)');
    ylabel('deg');
    grid

    titl=join(['q_', num2str(i), ' (t)']);
    title(titl)
end





%%

write_on_file


