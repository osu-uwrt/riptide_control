function [nlobj] = initialize(cfg)

configure_nmpc_codegen();
nlobj = nlmpc(13, 13, 8);
nlobj.Ts = cfg.Ts;
nlobj.PredictionHorizon = cfg.PH;
nlobj.ControlHorizon = cfg.CH;


nlobj.Model.IsContinuousTime = true;
% Order matches FossenBus and the arguments of FossenStateFcn.
nlobj.Model.NumberOfParameters =  11;
nlobj.Optimization.SolverOptions.MaxIter = 5;
nlobj.Optimization.UseSuboptimalSolution = true;
nlobj.Model.StateFcn = 'FossenStateFcn';
nlobj.Weights.OutputVariables = ones(1, 13);
nlobj.Optimization.CustomCostFcn = [];
nlobj.Optimization.ReplaceStandardCost = false;

end

