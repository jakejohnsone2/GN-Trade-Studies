function a = wrap_pi(a)
%WRAP_PI Wrap angles to (-pi, pi].
a = mod(a + pi, 2*pi) - pi;
end
