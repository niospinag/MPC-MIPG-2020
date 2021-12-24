function save_data(name,vhist,zhist,vphist,zphist,dhist,T)
% GET POSITIONS OF ALL AGENTS
% load("myFile1.mat")

Number_agents = size(vhist,1);
steps = size(vhist,2);
Start_position = [0; dhist(:,1)]- ones(Number_agents,1)*min(dhist(:,1));

agents = zeros(Number_agents, 2, steps);
agents(:,1,1)=Start_position;%initialize the positions
agents(:,2,:)=zhist;%lane is the same y pos

for j=1:Number_agents
    for i=2:steps
        agents(j,1,i) = agents(j,1,i-1) + vhist(j,i)*T; 
    end
end

agent1 = squeeze(agents(1,:,:));
agent2 = squeeze(agents(2,:,:));
agent3 = squeeze(agents(3,:,:));
agent4 = squeeze(agents(4,:,:));
agent5 = squeeze(agents(5,:,:));

save(name+'.mat','vhist','zhist','vphist','zphist','dhist','T', 'agents', 'agent1',...
    'agent2','agent3', 'agent4', 'agent5')
end
