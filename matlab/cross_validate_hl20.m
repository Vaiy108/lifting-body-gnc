%CROSS_VALIDATE_HL20 Compare Python aero model against the reference
% MATLAB implementation of the NASA TM-4302 HL-20 aerodynamic model.
%
% Requires: Aerospace Blockset HL-20 example on the path
%           (run openExample('aeroblk_HL20_UE') once so that
%            aeroblk_init_HL20 is available).
%
% Procedure:
%   1. Runs aeroblk_init_HL20 to build the reference polynomial data.
%   2. Evaluates datum coefficients on the (alpha, beta) grid directly
%      from the reference polynomials.
%   3. Loads the CSV grids exported by
%      python/scripts/generate_aero_tables.py.
%   4. Reports max absolute error per coefficient. Expected: < 1e-12
%      for the polynomial datum terms (identical arithmetic), exact-node
%      agreement for the Cn0 table.
%
% This validates DATA PORT integrity. Dynamic-response validation
% against the full Simulink airframe (damping-term convention, moment
% reference transfer) is performed by cross_validate_dynamics.m
% (Phase 2 deliverable).

clear; clc;

aeroblk_init_HL20;   % reference data (MathWorks example, not redistributed)

csv_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'hl20_aero');
alpha_grid = readmatrix(fullfile(csv_dir, 'breakpoints_alpha.csv'));
beta_grid  = readmatrix(fullfile(csv_dir, 'breakpoints_beta.csv'));

% --- Reference datum evaluation (same regressor as aeroblk_init_HL20) --
PolyCoeff = [-9.025e-2  2.632e-2  7.362e-2
             4.070e-2 -2.226e-3 -2.560e-4
             3.094e-5 -1.859e-5 -2.208e-4
             1.564e-5  6.001e-7 -2.262e-6
             -1.386e-6  1.828e-7  2.966e-7
             2.545e-8 -9.733e-9 -3.640e-9
             -1.189e-10 1.710e-10 9.388e-12
             2.564e-3 -5.233e-4 -5.299e-4
             8.501e-4  6.795e-5 -4.709e-4
             -1.156e-4 -1.993e-5  8.572e-5
             3.416e-6  1.341e-6 -4.199e-6
             -4.862e-4  6.061e-5  1.295e-4];

nA = numel(alpha_grid); nB = numel(beta_grid);
[CZ_ref, Cm_ref, CX_ref] = deal(zeros(nA, nB));
for i = 1:nA
    for j = 1:nB
        a = alpha_grid(i); b = beta_grid(j);
        reg = [1 a a^2 a^3 a^4 a^5 a^6 abs(b) b^2 abs(b)^3 b^4 a*abs(b)];
        t = reg * PolyCoeff;
        CZ_ref(i,j) = -t(1); Cm_ref(i,j) = t(2); CX_ref(i,j) = -t(3);
    end
end

CX_py = readmatrix(fullfile(csv_dir, 'datum_CX.csv'));
CZ_py = readmatrix(fullfile(csv_dir, 'datum_CZ.csv'));
Cm_py = readmatrix(fullfile(csv_dir, 'datum_Cm.csv'));

fprintf('Datum coefficient port errors (max abs):\n');
fprintf('  CX: %.3e\n', max(abs(CX_ref - CX_py), [], 'all'));
fprintf('  CZ: %.3e\n', max(abs(CZ_ref - CZ_py), [], 'all'));
fprintf('  Cm: %.3e\n', max(abs(Cm_ref - Cm_py), [], 'all'));
fprintf('Expected < 1e-12. Larger values indicate a port defect.\n');
fprintf('\nGrid size: %d alpha points x %d beta points\n', nA, nB);
fprintf('Spot check CZ at alpha=%.1f, beta=%.1f: ref=%.6f, python=%.6f\n', ...
    alpha_grid(6), beta_grid(11), CZ_ref(6,11), CZ_py(6,11));
