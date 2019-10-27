
function def_condinicial(zel, d1i,Dl,alogic1,alogic2,blogic1,blogic2,G1logic,G2logic,S1logic,S2logic,N1logic,N2logic);

% hold on
% vhist = vel;
% zhist = zel;
% ahist = past_a;
% dhist = d1i;
%......A logicos  
if zel(1) == zel(2)
       alogic1=[1];
  else
       alogic1=[0];
end

if zel(1) == zel(3)
       alogic2=[1];
  else
       alogic2=[0];
end   

  %......B logicos  
if d1i(1)  >= 0
       blogic1=[1];
  else
       blogic1=[0];
end

if d1i(1) >= 0
       blogic2=[1];
  else
       blogic2=[0];
end   

%......G logicos  

 G1logic=[1]; G2logic=[1];
 
%......S logicos 
if zel(1) - zel(2) == 1
       S1logic=[1];
  else
       S1logic=[0];
end

if zel(1) - zel(3) == 1
       S2logic=[1];
  else
       S2logic=[0];
end   

  %......N logicos  
if (d1i(1)  <= Dl)||(d1i(1)  >= -Dl)
       N1logic=[1];
  else
       N1logic=[0];
end

if (d1i(2)  <= Dl)||(d1i(2)  >= -Dl)
       N2logic=[1];
  else
       N2logic=[0];
end    

