function fun = getEye(q)
    %% Get function handle for identity matrix of appropriate class
    if isa(q, "double")
        fun = @eye;
    elseif isa(q, "casadi.MX")
        fun = @casadi.MX.eye;
    elseif isa(q, "casadi.SX")
        fun = @casadi.SX.eye;
    elseif isa(q, "casadi.DM")
        fun = @casadi.DM.eye;
    end
end
