function [phi] = statetrans(A)
   t = sym('t');
   phi = expm(A * t);
end