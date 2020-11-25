function [a]=influenceu(distance,pow)

a=1./((1+distance.^2).^pow);

end

