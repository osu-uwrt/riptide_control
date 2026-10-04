function [nlobj] = initialize(cfg)


nlobj = nlmpc(13, 13, 8);
nlobj.Ts = cfg.Ts;
nlobj.PredictionHorizon = cfg.PH;
nlobj.ControlHorizon = cfg.CH;

nlobj.Model.IsContinuousTime = true;
nlobj.Model.NumberOfParameters = 11;
nlobj.Model.StateFcn = 'FossenStateFcn';
nlobj.Weights.OutputVariables = ones(1, 13);
nlobj.Optimization.CustomCostFcn = [];
nlobj.Optimization.ReplaceStandardCost = false;

end
