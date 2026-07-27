import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (side-effect import)

from modules.common.turn_dynamics import turn_radius_from_bank_m

# ────────────────────────── 로컬 함수들 ──────────────────────────
def get_dta(V: float, max_roll: float, leg_deg: float, dlim: float) -> float:
    """
    DTA(distance-to-alternate) 계산
    V        : 비행 속도 (m/s)
    max_roll : 최대 롤각 (deg)
    leg_deg  : 헤딩 변화각 (deg)
    dlim     : 최대 거리 제한 (m)
    """
    # 최대 기울기 제한
    max_roll = min(max_roll, V * 0.9719)
    max_roll = min(max_roll, abs(leg_deg) * 0.5)

    # 회전 반경
    Rv = turn_radius_from_bank_m(V, max_roll)
    if Rv is None:
        return float(dlim)

    # DTA 계산
    dta = Rv * min(np.tan(np.deg2rad(abs(leg_deg) * 0.5)), 8) + 3 * V

    # 최대 거리 제한
    return min(dta, dlim)


def get_radius(V: float, max_roll: float) -> float:
    """최소 회전 반경 계산"""
    radius = turn_radius_from_bank_m(V, max_roll)
    return float(radius) if radius is not None else float("inf")


# ────────────────────────── 메인 로직 ──────────────────────────
def main():
    # 파라미터
    num_leg   = 500
    leg       = np.linspace(-170, 170, num_leg)     # heading(°)
    num_V     = 10
    V_arr     = np.linspace(28, 55, num_V)          # speed (m/s)
    MAX_ROLL0 = 30.0                                # 최대 롤각(°)
    DLIM      = 800                                 # 거리 상한(m)

    # 계산 배열
    Dta = np.zeros((num_V, num_leg))
    R   = np.zeros((num_V, num_leg))

    for i, V in enumerate(V_arr):
        for j, lg in enumerate(leg):
            Dta[i, j] = get_dta(V, MAX_ROLL0, lg, DLIM)
            R[i, j]   = get_radius(V, MAX_ROLL0)

    # 2-D subplot
    plt.close("all")
    fig2d = plt.figure(figsize=(14, 6), num='DTA vs Heading for Each Speed')
    rows, cols = 2, 5

    for i, V in enumerate(V_arr, start=1):
        ax = fig2d.add_subplot(rows, cols, i)
        ax.plot(leg, Dta[i-1, :], 'r-', linewidth=1, label='DTA')
        ax.plot(leg, R[i-1, :],   'b-', linewidth=1, label='MinTurnRadius')
        ax.grid(which='both', linestyle='--', linewidth=0.5)
        ax.set_title(f'V = {V:.1f} m/s')
        ax.set_xlabel('Heading (deg)')
        ax.set_ylabel('Distance (m)')
        if i == 1:
            ax.legend(loc='best')

    fig2d.tight_layout()

    # 3-D surfaces
    Xgrid, Vgrid = np.meshgrid(leg, V_arr)
    fig3d = plt.figure(figsize=(14, 6), num='3D Surfaces: DTA & MinTurnRadius')

    ax1 = fig3d.add_subplot(1, 2, 1, projection='3d')
    surf1 = ax1.plot_surface(Xgrid, Vgrid, Dta, edgecolor='none', cmap='viridis')
    ax1.set_xlabel('Heading (deg)')
    ax1.set_ylabel('Speed (m/s)')
    ax1.set_zlabel('DTA (m)')
    ax1.set_title('DTA Surface')
    fig3d.colorbar(surf1, ax=ax1, shrink=0.6)
    ax1.view_init(elev=30, azim=45)

    ax2 = fig3d.add_subplot(1, 2, 2, projection='3d')
    surf2 = ax2.plot_surface(Xgrid, Vgrid, R, edgecolor='none', cmap='plasma')
    ax2.set_xlabel('Heading (deg)')
    ax2.set_ylabel('Speed (m/s)')
    ax2.set_zlabel('Min Turn Radius (m)')
    ax2.set_title('Min Turn Radius Surface')
    fig3d.colorbar(surf2, ax=ax2, shrink=0.6)
    ax2.view_init(elev=30, azim=45)

    plt.show()


if __name__ == "__main__":
    main()
