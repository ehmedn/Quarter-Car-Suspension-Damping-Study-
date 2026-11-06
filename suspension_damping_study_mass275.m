%% Quarter car suspension study
% Passive suspension model with two masses, a spring and a damper.
% Compares tyre load, body acceleration and suspension travel.
% Vehicle and road parameters are assumed values for this study.
%
% Model equations adapted from the MathWorks quarter car example:
% https://www.mathworks.com/help/mpc/ug/admm-based-mpc-control-for-quarter-car-suspension.html

clear; clc; close all;

%% 1. Model parameters in SI units
study_tag = 'mass275';       % Change to 'my_sensitivity' for your second run.
p.ms = 275;                   % Sprung mass allocated to one corner, kg.
p.mu = 35;                    % Wheel/assembly unsprung mass, kg.
p.ks = 18000;                 % Suspension spring stiffness, N/m.
p.kt = 180000;                % Tyre vertical stiffness, N/m.
p.g = 9.81;                   % Gravitational acceleration, m/s^2.
p.bump_height = 0.025;        % Peak height of the assumed bump, m (25 mm).
p.bump_length = 3;            % Length along the road, m.
p.bump_start = 0.5;           % Time when the wheel meets the bump, s.
p.t_end = 3;                  % Same end time for every case, s.
p.sample_dt = 0.001;          % Output sample interval, s.
p.stroke_limit = 0.040;       % Assumed maximum dynamic suspension travel, m.
c_values = [500; 1000; 1500; 2000; 3000]; % Damper coefficients, N*s/m.
speeds = [15; 20];            % Two speeds, m/s (54 and 72 km/h).
baseline_c = 1000;            % Comparison baseline, N*s/m.

assert(all([p.ms p.mu p.ks p.kt p.g p.bump_length p.sample_dt] > 0));
assert(p.bump_height >= 0 && all(c_values > 0) && all(speeds > 0));
assert(sum(c_values == baseline_c) == 1, 'Keep the baseline in c_values.');
assert(p.t_end > p.bump_start + p.bump_length/min(speeds));
assert(all(isfinite(c_values)) && all(isfinite(speeds)));

project_folder = fileparts(mfilename('fullpath'));
if isempty(project_folder), project_folder = pwd; end
out_folder = fullfile(project_folder, ['results_' study_tag]);
if ~exist(out_folder, 'dir'), mkdir(out_folder); end
options = odeset('RelTol',1e-7,'AbsTol',1e-9,'MaxStep',0.0025);
N_static = (p.ms + p.mu)*p.g;

%% 2. Hand calculations
body_frequency_Hz = sqrt(p.ks/p.ms)/(2*pi); % Wheel held fixed approximation.
critical_damping_Nsm = 2*sqrt(p.ks*p.ms);   % Same one-mass approximation.
baseline_damping_ratio = baseline_c/critical_damping_Nsm;
fprintf('Approximate body natural frequency: %.3f Hz\n',body_frequency_Hz);
fprintf('Approximate baseline damping ratio: %.3f\n',baseline_damping_ratio);
fprintf('Static vertical load at this corner: %.2f N\n',N_static);

%% 3. Equilibrium check on a flat road
flat = p; flat.bump_height = 0;
flat_case = simulate_case(flat,baseline_c,speeds(1),options);
zero_road_max_state = max(abs(flat_case.x(:)));
assert(zero_road_max_state < 1e-10, 'Zero-road equilibrium check failed.');

%% 4. Damping comparison at two speeds
% State x = [body displacement; body velocity; wheel displacement; wheel velocity].
% Displacements are measured from the static equilibrium, so gravity does
% not appear again in the dynamic equations.
all_cases = cell(numel(speeds),numel(c_values));
all_tables = cell(numel(speeds),1);
chosen_indices = nan(numel(speeds),1);
best_c = nan(numel(speeds),1);
baseline_load_rms_pct = nan(numel(speeds),1);
selected_load_rms_pct = nan(numel(speeds),1);
selected_accel_rms = nan(numel(speeds),1);
improvement_pct = nan(numel(speeds),1);
baseline_index = find(c_values == baseline_c,1);

for s = 1:numel(speeds)
    load_rms_pct = zeros(numel(c_values),1);
    accel_rms = load_rms_pct; peak_accel = load_rms_pct;
    peak_travel_mm = load_rms_pct; min_load_N = load_rms_pct;
    for j = 1:numel(c_values)
        q = simulate_case(p,c_values(j),speeds(s),options);
        all_cases{s,j} = q;
        load_rms_pct(j) = q.metrics(1);
        accel_rms(j) = q.metrics(2);
        peak_accel(j) = q.metrics(3);
        peak_travel_mm(j) = q.metrics(4);
        min_load_N(j) = q.metrics(5);
    end
    passes_screen = peak_travel_mm <= 1000*p.stroke_limit & min_load_N > 0;
    assert(all(isfinite([load_rms_pct; accel_rms; peak_accel; peak_travel_mm; min_load_N])));
    T = table(c_values,load_rms_pct,accel_rms,peak_accel,peak_travel_mm,min_load_N,passes_screen, ...
        'VariableNames',{'Damping_Nsm','TyreLoadRMS_pctStatic','BodyAccelRMS_ms2', ...
        'PeakBodyAccel_ms2','PeakDynamicTravel_mm','MinimumTyreLoad_N','PassesStudyScreen'});
    all_tables{s} = T;
    fprintf('\nRESULTS at %.1f m/s -- metrics use %.1f to %.1f s\n', ...
        speeds(s),p.bump_start,p.t_end);
    disp(T);
    writetable(T,fullfile(out_folder,sprintf('comparison_%gmps.csv',speeds(s))));

    % Select the lowest RMS tyre-load variation within the travel limit,
    % with positive modelled tyre load.
    score = load_rms_pct; score(~passes_screen) = Inf;
    [best_score,jbest] = min(score);
    baseline_load_rms_pct(s) = load_rms_pct(baseline_index);
    if isfinite(best_score)
        chosen_indices(s) = jbest; best_c(s) = c_values(jbest);
        selected_load_rms_pct(s) = best_score;
        selected_accel_rms(s) = accel_rms(jbest);
        improvement_pct(s) = 100*(baseline_load_rms_pct(s)-best_score)/baseline_load_rms_pct(s);
        fprintf('Lowest-load-variation passing candidate: c = %g N*s/m.\n',best_c(s));
        fprintf('RMS tyre-load variation reduction vs baseline: %.2f%%.\n',improvement_pct(s));
    else
        fprintf('No candidate passed. Report this; do not call any case acceptable.\n');
    end

    % Plot the response for each damping setting.
    fig = figure('Color','w','Position',[100 100 1100 760]);
    labels = arrayfun(@(c)sprintf('c = %g N s/m',c),c_values,'UniformOutput',false);
    subplot(2,2,1); plot(all_cases{s,1}.t,1000*all_cases{s,1}.road,'k','LineWidth',1.5);
    grid on; xlabel('Time (s)'); ylabel('Road height (mm)');
    title(sprintf('Assumed %g mm bump at %g m/s',1000*p.bump_height,speeds(s)));
    subplot(2,2,2); hold on;
    for j=1:numel(c_values), q=all_cases{s,j}; plot(q.t,q.body_accel,'LineWidth',1.1); end
    grid on; xlabel('Time (s)'); ylabel('Body acceleration (m/s^2)');
    title('Vertical body response'); legend(labels,'Location','northeast');
    subplot(2,2,3); hold on;
    for j=1:numel(c_values), q=all_cases{s,j}; plot(q.t,100*q.tyre_load/N_static,'LineWidth',1.1); end
    plot([0 p.t_end],[0 0],'k:');
    grid on; xlabel('Time (s)'); ylabel('Tyre load (% of static load)');
    title('Modelled vertical tyre load');
    subplot(2,2,4); hold on;
    for j=1:numel(c_values), q=all_cases{s,j}; plot(q.t,1000*q.stroke,'LineWidth',1.1); end
    plot([0 p.t_end],1000*p.stroke_limit*[1 1],'k:');
    plot([0 p.t_end],-1000*p.stroke_limit*[1 1],'k:');
    grid on; xlabel('Time (s)'); ylabel('Dynamic suspension travel (mm)');
    title('Body to wheel relative displacement');
    print(fig,fullfile(out_folder,sprintf('response_%gmps.png',speeds(s))),'-dpng','-r180');
end

%% 5. Damping tradeoff and speed comparison
fig = figure('Color','w','Position',[100 100 1100 450]);
for s=1:numel(speeds)
    subplot(1,numel(speeds),s); hold on; T=all_tables{s};
    plot(T.TyreLoadRMS_pctStatic,T.BodyAccelRMS_ms2,'o-','LineWidth',1.2);
    for j=1:numel(c_values)
        text(T.TyreLoadRMS_pctStatic(j),T.BodyAccelRMS_ms2(j), ...
            sprintf('  %g',c_values(j)),'FontSize',10);
    end
    grid on; xlabel('RMS tyre-load variation (% of static load)');
    ylabel('RMS body acceleration (m/s^2)'); title(sprintf('%g m/s; labels are c in N s/m',speeds(s)));
end
print(fig,fullfile(out_folder,'damping_tradeoff.png'),'-dpng','-r180');
summary = table(speeds,best_c,baseline_load_rms_pct,selected_load_rms_pct, ...
    improvement_pct,selected_accel_rms,'VariableNames',{'Speed_ms','SelectedDamping_Nsm', ...
    'BaselineTyreLoadRMS_pctStatic','SelectedTyreLoadRMS_pctStatic', ...
    'ReductionVsBaseline_pct','SelectedBodyAccelRMS_ms2'});
writetable(summary,fullfile(out_folder,'speed_sensitivity.csv'));

%% 6. Numerical convergence check
% This checks numerical consistency. It is not validation against a real car.
metric_change_pct = nan(numel(speeds),5);
refined = p; refined.sample_dt = p.sample_dt/2;
fine_options = odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.00125);
for s=1:numel(speeds)
    if ~isnan(chosen_indices(s))
        q0 = all_cases{s,chosen_indices(s)};
        q1 = simulate_case(refined,best_c(s),speeds(s),fine_options);
        metric_change_pct(s,:) = 100*abs(q1.metrics-q0.metrics)./max(abs(q1.metrics),1e-12);
    end
end
numerical_checks = table(speeds,metric_change_pct(:,1),metric_change_pct(:,2), ...
    metric_change_pct(:,3),metric_change_pct(:,4),metric_change_pct(:,5), ...
    'VariableNames',{'Speed_ms','TyreRMS_change_pct','AccelRMS_change_pct', ...
    'PeakAccel_change_pct','Travel_change_pct','MinLoad_change_pct'});
writetable(numerical_checks,fullfile(out_folder,'numerical_checks.csv'));
disp(summary); disp(numerical_checks);
if any(metric_change_pct(:)>1)
    warning('A metric changed by more than 1%%. Resolve this before using that result.');
end

%% 7. Save model inputs and results
save(fullfile(out_folder,'study_run.mat'),'p','c_values','speeds','baseline_c', ...
    'all_tables','summary','numerical_checks','zero_road_max_state','all_cases');
fid = fopen(fullfile(out_folder,'run_notes.txt'),'w');
if fid ~= -1
    fprintf(fid,'Run: %s\nStudy tag: %s\n',datestr(now,31),study_tag);
    fprintf(fid,'Assumed ms=%g kg, mu=%g kg, ks=%g N/m, kt=%g N/m\n',p.ms,p.mu,p.ks,p.kt);
    fprintf(fid,'Assumed bump: height=%g m, length=%g m\n',p.bump_height,p.bump_length);
    fprintf(fid,'Assumed dynamic travel limit: %g m\n',p.stroke_limit);
    fprintf(fid,'Metrics window: %g to %g s\n',p.bump_start,p.t_end);
    fprintf(fid,'Zero-road maximum absolute state: %.3g\n',zero_road_max_state);
    fprintf(fid,'All parameters are study assumptions. No track or physical test data used.\n');
    fprintf(fid,'Best means lowest RMS tyre-load variation among tested passing cases only.\n');
    fclose(fid);
end
fprintf('\nFiles saved to:\n%s\n',out_folder);
fprintf('Now interpret both speeds, make one documented input change, and write your report.\n');

%% Local functions
function q = simulate_case(p,c,speed,options)
    t_grid = (0:p.sample_dt:p.t_end)';
    [t,x] = ode45(@(t,x)quarter_car_rhs(t,x,p,c,speed),t_grid,zeros(4,1),options);
    road = road_bump(t,p,speed);
    stroke = x(:,1)-x(:,3);
    relative_velocity = x(:,2)-x(:,4);
    body_accel = (-p.ks*stroke-c*relative_velocity)/p.ms;
    N_static = (p.ms+p.mu)*p.g;
    delta_load = p.kt*(road-x(:,3));
    tyre_load = N_static+delta_load;
    in_window = t>=p.bump_start & t<=p.t_end;
    tw=t(in_window); duration=tw(end)-tw(1);
    rms_load=sqrt(trapz(tw,delta_load(in_window).^2)/duration);
    rms_accel=sqrt(trapz(tw,body_accel(in_window).^2)/duration);
    q.t=t; q.x=x; q.road=road; q.stroke=stroke;
    q.body_accel=body_accel; q.tyre_load=tyre_load;
    q.metrics=[100*rms_load/N_static, rms_accel, max(abs(body_accel(in_window))), ...
        1000*max(abs(stroke(in_window))), min(tyre_load(in_window))];
end

function dx = quarter_car_rhs(t,x,p,c,speed)
    % Upward displacement is positive. Both spring/damper forces oppose
    % relative motion and act in equal and opposite directions on the masses.
    zr=road_bump(t,p,speed);
    spring_force=p.ks*(x(1)-x(3));
    damper_force=c*(x(2)-x(4));
    body_accel=(-spring_force-damper_force)/p.ms;
    wheel_accel=(spring_force+damper_force-p.kt*(x(3)-zr))/p.mu;
    dx=[x(2); body_accel; x(4); wheel_accel];
end

function zr = road_bump(t,p,speed)
    % Smooth raised-cosine profile: zero at both ends, peak = bump_height.
    duration=p.bump_length/speed;
    elapsed=t-p.bump_start;
    inside=elapsed>=0 & elapsed<=duration;
    zr=zeros(size(t));
    zr(inside)=p.bump_height/2*(1-cos(2*pi*elapsed(inside)/duration));
end
