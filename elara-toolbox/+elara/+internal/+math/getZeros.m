function fun = getZeros(q)
    %% Get function handle for zeros function of appropriate class
    if isa(q, "double")
        fun = @zeros;
    elseif isa(q, "casadi.MX")
        fun = @casadi.MX.zeros;
    elseif isa(q, "casadi.SX")
        fun = @casadi.SX.zeros;
    elseif isa(q, "casadi.DM")
        fun = @casadi.DM.zeros;
    end
end
