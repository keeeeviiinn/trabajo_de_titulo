function [f]=repulsion(distance,pow)

f=1./(distance-1e-10).^pow;
end