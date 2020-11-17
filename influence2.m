function [a]=influence2(distance,pow)
global sigma
if distance==0
    a=0;
else
    a=distance./((1+sigma+distance).^pow);
end
end