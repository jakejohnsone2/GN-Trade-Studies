function x = dnc_plant_init_stub(start_pose, p)
%DNC_PLANT_INIT_STUB Initial plant state for the stand-in plant.
%   Owner: D&C. State is [x; y; theta; v; omega]; any D&C state
%   is fine as long as x(1:3) is the pose.
x = [start_pose(:); 0; 0];
end
