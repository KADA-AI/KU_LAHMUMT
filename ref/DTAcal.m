clc; clear; close all;

%% 파라미터 설정
numleg   = 500;
leg      = linspace(-170, 170, numleg);
numV     = 10;
V        = linspace(28, 55, numV);
MaxRoll0 = 30.0;
dlim     = 800;

%% 거리 계산
Dta = zeros(numV, numleg);
R   = zeros(numV, numleg);
for i = 1:numV
    for j = 1:numleg
        Dta(i,j) = getDTA( V(i), MaxRoll0, leg(j), dlim );
        R(i,j)   = getRadius( V(i), MaxRoll0 );
    end
end

%% 2D subplot
figure('Name','DTA vs Heading for Each Speed','NumberTitle','off');
rows = 2; cols = 5;
for i = 1:numV
    ax = subplot(rows, cols, i);
    plot(leg, Dta(i,:), 'r-', 'LineWidth',1); hold(ax,'on');
    plot(leg, R(i,:),   'b-', 'LineWidth',1);
    grid(ax,'minor');
    title(ax, sprintf('V = %.1f m/s', V(i)));
    xlabel(ax,'Heading (deg)');
    ylabel(ax,'Distance (m)');
    if i==1
        legend(ax, 'DTA', 'MinTurnRadius', 'Location','best');
    end
end

%% 3D surface subplot
[Xgrid, Vgrid] = meshgrid(leg, V);

figure('Name','3D Surfaces: DTA & MinTurnRadius','NumberTitle','off');

% DTA surface
subplot(1,2,1);
surf(Xgrid, Vgrid, Dta, 'EdgeColor','none');
xlabel('Heading (deg)');
ylabel('Speed (m/s)');
zlabel('DTA (m)');
title('DTA Surface');
colorbar;
view(45,30);

% MinTurnRadius surface
subplot(1,2,2);
surf(Xgrid, Vgrid, R, 'EdgeColor','none');
xlabel('Heading (deg)');
ylabel('Speed (m/s)');
zlabel('Min Turn Radius (m)');
title('Min Turn Radius Surface');
colorbar;
view(45,30);


%% 로컬 함수 정의
function dta = getDTA(V, MaxRoll, leg, dlim)
    % V       : 비행속도 (m/s) 
    % MaxRoll : 최대 롤각도 (deg)
    % leg     : leg 변화각 (deg)
    % dlim    : 반경제한 (m)

    % 최대 기울기 제한
    MaxRoll = min(MaxRoll, V*0.9719);
    MaxRoll = min(MaxRoll, abs(leg)*0.5);
    % 회전 반경
    Rv = V^2 / (9.8 * tand(MaxRoll));
    % DTA 계산
    dta = Rv * min(tand(abs(leg)*0.5), 8) + 3 * V;
    % 최대 거리 제한
    dta = min(dta, dlim);
end

function Rv = getRadius(V, MaxRoll)
    % V       : 비행속도 (m/s) 
    % MaxRoll : 최대 롤각도 (deg)
    Rv = V^2 / (9.8 * tand(MaxRoll));
end
