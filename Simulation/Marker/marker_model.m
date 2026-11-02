function [m_est, Sigma] = marker_model(m_true, p)
%MARKER_MODEL Marker position as the UAS reports it (GN.2a, IF-UAS.1).
%   m_true : true marker position [x, y] (m)
%   m_est  : reported marker position, 2x1 (m)
%   Sigma  : reported 2x2 covariance (m^2)
%
%   Modes (p.marker.mode)
%     'worst_ring' offset of exactly p.marker.bound at p.marker.ring_angle.
%                  Sweep the angle to verify against the hard bound.
%     'gaussian'   Gaussian with sigma p.marker.sigma, truncated at the
%                  bound because the UAS treats 15 cm as hard.
%     'uniform'    uniform over the disk of radius p.marker.bound.

m_true = m_true(:);
R      = p.marker.bound;

switch p.marker.mode
    case 'worst_ring'
        a      = p.marker.ring_angle;
        offset = R * [cos(a); sin(a)];
    case 'gaussian'
        offset = p.marker.sigma * randn(2,1);
        while norm(offset) > R
            offset = p.marker.sigma * randn(2,1);
        end
    case 'uniform'
        r      = R * sqrt(rand);
        a      = 2*pi*rand;
        offset = r * [cos(a); sin(a)];
    otherwise
        error('marker_model:mode', 'Unknown marker mode "%s".', p.marker.mode);
end

m_est = m_true + offset;
Sigma = p.marker.sigma^2 * eye(2);
end
