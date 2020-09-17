function [a]=influence(distance,pow)
global sigma
if distance==0
    a=0;
else
    a=1./((sigma+distance).^pow);
end
end