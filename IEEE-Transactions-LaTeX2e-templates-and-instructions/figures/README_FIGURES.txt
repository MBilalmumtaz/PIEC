% FIGURE PLACEHOLDERS
% ===================
% This directory contains figures for the PIEC TEVC paper.
% Replace the placeholder descriptions below with actual figures
% before final submission.
%
% Required figures:
%
% fig_system.pdf / fig_system.png
%   - System architecture of RT-NSGA-II framework
%   - Shows: local map + UKF state estimator -> RT-NSGA-II planner ->
%     Pareto front -> reactive local controller -> UGV
%
% fig_warmstart.pdf / fig_warmstart.png
%   - Warm-start population initialization concept
%   - Shows: previous Pareto front F*_{k-1} seeding current generation P_0^k
%   - Include objective-space scatter plot contrasting warm vs cold start
%
% fig_surrogate.pdf / fig_surrogate.png
%   - Surrogate model architecture (3-layer FNN: 10->64->64->1)
%   - Training loss curves and surrogate vs. simulation accuracy
%   - Parity plot (predicted vs ground-truth energy)
%
% fig_convergence.pdf / fig_convergence.png
%   - Hypervolume convergence curves: RT-NSGA-II vs. cold-start NSGA-II
%   - X-axis: number of fitness evaluations, Y-axis: hypervolume indicator
%   - Show statistical bands (mean +/- std over 30 runs)
%
% fig_pareto.pdf / fig_pareto.png
%   - 2D projection of Pareto front: energy vs. obstacle clearance
%   - Overlaid for RT-NSGA-II, standard NSGA-II, and A* solution
%
% fig_realworld.pdf / fig_realworld.png
%   - Real-world Scout Mini UGV deployment photos
%   - Annotated trajectory overlays in three test environments
%   - Energy meter traces comparing PIEC vs. A*+PID
%
% fig_ablation.pdf / fig_ablation.png
%   - Bar charts showing ablation: full RT-NSGA-II vs. no warm-start,
%     no surrogate, cold-start, each component removed
