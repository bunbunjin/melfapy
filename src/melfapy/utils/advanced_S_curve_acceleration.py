import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

# Matplotlibの日本語フォント設定
# ご使用の環境に日本語フォントがない場合は、適宜インストールしてください。
# (例: 'IPAexGothic', 'Yu Gothic', 'Meiryo')
try:
    mpl.rcParams["font.sans-serif"] = [
        "Hiragino Maru Gothic Pro",
        "Yu Gothic",
        "MS Gothic",
    ]
    mpl.rcParams["axes.unicode_minus"] = False  # マイナス記号の文字化けを防ぐ
except Exception as e:
    print(f"日本語フォントの設定中にエラーが発生しました: {e}")
    print("グラフのラベルが正しく表示されない可能性があります。")


class AdvancedSCurvePlanner:
    """
    アドバンストS字加減速（7区間）プロファイルを生成するクラス。

    このクラスは、指定された移動の制約条件に基づき、ジャークが一定となる
    7つの区間（加速フェーズ3区間、定速フェーズ1区間、減速フェーズ3区間）
    から成るモーションプロファイルを計算します。
    """

    def __init__(self, q0, q1, v_max, a_max, j_max):
        """
        パラメータを初期化し、モーションプロファイルの時間を計算します。

        Args:
            q0 (float): 開始位置
            q1 (float): 終了位置
            v_max (float): 最大速度 ( > 0)
            a_max (float): 最大加速度 ( > 0)
            j_max (float): 最大ジャーク（加々速度） ( > 0)
        """
        self.q0 = q0
        self.q1 = q1
        self.sign = np.sign(q1 - q0)

        # 移動方向に関わらず計算を単純化するため、全ての制約を正の値とする
        self.v_max = abs(v_max)
        self.a_max = abs(a_max)
        self.j_max = abs(j_max)

        # 総移動距離
        self.D = abs(q1 - q0)

        # 移動距離がゼロの場合は計算をスキップ
        if self.D == 0:
            self.Tj1, self.Ta, self.Tj2, self.Tv = 0, 0, 0, 0
            self.v_lim, self.a_lim = 0, 0
            self.T = 0
            return

        # --- 各区間の時間を計算 ---
        # Case 1: 最大加速度・最大速度に到達する (速度プロファイルが台形)
        if (self.v_max * self.j_max) >= (self.a_max**2):
            self.Tj1 = self.a_max / self.j_max
            self.Ta = self.v_max / self.a_max - self.Tj1

            # 定速区間(Tv)が存在するかチェック
            if (self.D * self.j_max**2) >= (
                self.a_max * (self.a_max**2 + 2 * self.v_max * self.j_max)
            ):
                self.Tv = self.D / self.v_max - (
                    self.a_max / self.j_max + self.v_max / self.a_max
                )
                self.v_lim = self.v_max
                self.a_lim = self.a_max
                self.Tj2 = self.Tj1
            # Case 2: 最大速度には到達しない (速度プロファイルが三角形)
            else:
                self.Tj1 = (
                    np.sqrt(4 * self.D * self.j_max**2 + self.a_max**3) - self.a_max**2
                ) / (2 * self.a_max * self.j_max)
                self.Ta = 0
                self.Tv = 0
                self.a_lim = self.j_max * self.Tj1
                self.v_lim = self.a_lim * self.Tj1
                self.Tj2 = self.Tj1
        # Case 3: 最大加速度に到達しない (加速度プロファイルが三角形)
        else:
            self.Tj1 = (self.D / (2 * self.j_max)) ** (1 / 3)
            self.Ta = 0
            self.v_lim = self.j_max * self.Tj1**2

            # 定速区間(Tv)が存在するかチェック
            if self.v_lim > self.v_max:
                self.Tj1 = np.sqrt(self.v_max / self.j_max)
                self.Tv = self.D / self.v_max - 2 * self.Tj1
                self.v_lim = self.v_max
            else:
                self.Tv = 0

            self.a_lim = self.j_max * self.Tj1
            self.Tj2 = self.Tj1

        # 総移動時間
        self.T = 2 * self.Tj1 + self.Ta + self.Tv + 2 * self.Tj2 + self.Ta

        # 各フェーズの終了時刻
        self.t1 = self.Tj1
        self.t2 = self.t1 + self.Ta
        self.t3 = self.t2 + self.Tj2
        self.t4 = self.t3 + self.Tv
        self.t5 = self.t4 + self.Tj2
        self.t6 = self.t5 + self.Ta
        self.t7 = self.t6 + self.Tj1
        self.T = self.t7  # 全移動時間

    def get_profile(self, t):
        """
        指定された時刻 t における状態（位置、速度、加速度、ジャーク）を返します。

        Args:
            t (float): 時刻

        Returns:
            tuple: (位置, 速度, 加速度, ジャーク)
        """
        j_max_signed = self.sign * self.j_max
        a_lim_signed = self.sign * self.a_lim
        v_lim_signed = self.sign * self.v_lim

        # 加速フェーズ
        if 0 <= t < self.t1:  # Phase 1: ジャーク増加
            jerk = j_max_signed
            acc = j_max_signed * t
            vel = 0.5 * j_max_signed * t**2
            pos = self.q0 + (1 / 6) * j_max_signed * t**3
        elif self.t1 <= t < self.t2:  # Phase 2: 定加速度
            jerk = 0
            acc = a_lim_signed
            vel = a_lim_signed * (t - 0.5 * self.Tj1)
            pos = self.q0 + a_lim_signed * (
                0.5 * t**2 - 0.5 * self.Tj1 * t + (1 / 6) * self.Tj1**2
            )
        elif self.t2 <= t < self.t3:  # Phase 3: ジャーク減少
            t_rem = self.t3 - t
            jerk = -j_max_signed
            acc = j_max_signed * t_rem
            vel = v_lim_signed - 0.5 * j_max_signed * t_rem**2
            pos_t3 = self.q0 + v_lim_signed * (self.t3 - self.Tj1 - 0.5 * self.Ta)
            pos = pos_t3 - (v_lim_signed * t_rem - (1 / 6) * j_max_signed * t_rem**3)

        # 定速フェーズ
        elif self.t3 <= t < self.t4:  # Phase 4: 定速
            jerk = 0
            acc = 0
            vel = v_lim_signed
            pos_t3 = self.q0 + v_lim_signed * (self.t3 - self.Tj1 - 0.5 * self.Ta)
            pos = pos_t3 + v_lim_signed * (t - self.t3)

        # 減速フェーズ
        elif self.t4 <= t < self.t5:  # Phase 5: 減速開始（ジャーク増加）
            t_rem = t - self.t4
            jerk = -j_max_signed
            acc = -j_max_signed * t_rem
            vel = v_lim_signed - 0.5 * j_max_signed * t_rem**2
            pos_t4 = (
                self.q0
                + v_lim_signed * (self.t3 - self.Tj1 - 0.5 * self.Ta)
                + v_lim_signed * self.Tv
            )
            pos = pos_t4 + v_lim_signed * t_rem - (1 / 6) * j_max_signed * t_rem**3
        elif self.t5 <= t < self.t6:  # Phase 6: 定減速
            t_rem = t - self.t5
            jerk = 0
            acc = -a_lim_signed
            vel = v_lim_signed - a_lim_signed * (self.Tj2 + t_rem)
            pos_t5 = (
                self.q0
                + v_lim_signed
                * (self.t3 - self.Tj1 - 0.5 * self.Ta + self.Tv + self.Tj2)
                - (1 / 6) * j_max_signed * self.Tj2**3
            )
            pos = (
                pos_t5
                + (v_lim_signed - 0.5 * a_lim_signed * self.Tj2) * t_rem
                - 0.5 * a_lim_signed * t_rem**2
            )
        elif self.t6 <= t <= self.t7:  # Phase 7: 減速終了（ジャーク減少）
            t_rem = self.t7 - t
            jerk = j_max_signed
            acc = j_max_signed * t_rem
            vel = 0.5 * j_max_signed * t_rem**2
            pos = self.q1 - (1 / 6) * j_max_signed * t_rem**3

        # 範囲外の時刻
        else:
            jerk = 0
            acc = 0
            if t < 0:
                vel = 0
                pos = self.q0
            else:
                vel = 0
                pos = self.q1

        return pos, vel, acc, jerk


def plot_profiles(planner, title):
    """
    生成されたプロファイルをプロットします。
    """
    total_time = planner.T
    if total_time == 0:
        print(f"{title}: 移動距離がゼロのため、プロットをスキップします。")
        return

    # 時間配列を生成
    t_array = np.linspace(0, total_time, 1000)

    # 各時刻の状態を計算
    pos_array, vel_array, acc_array, jerk_array = [], [], [], []
    for t in t_array:
        pos, vel, acc, jerk = planner.get_profile(t)
        pos_array.append(pos)
        vel_array.append(vel)
        acc_array.append(acc)
        jerk_array.append(jerk)

    # プロット
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    fig.suptitle(f"アドバンストS字加減速プロファイル\n({title})", fontsize=16)

    # 位置
    axs[0].plot(t_array, pos_array, "b")
    axs[0].set_ylabel("位置 (q)")
    axs[0].grid(True)
    axs[0].set_title(f"総移動時間: {total_time:.3f} s")

    # 速度
    axs[1].plot(t_array, vel_array, "g")
    axs[1].set_ylabel("速度 (v)")
    axs[1].axhline(
        y=planner.v_lim * planner.sign,
        color="r",
        linestyle="--",
        label=f"v_lim={planner.v_lim:.2f}",
    )
    axs[1].axhline(y=-planner.v_lim * planner.sign, color="r", linestyle="--")
    axs[1].grid(True)
    axs[1].legend(loc="best")

    # 加速度
    axs[2].plot(t_array, acc_array, "r")
    axs[2].set_ylabel("加速度 (a)")
    axs[2].axhline(
        y=planner.a_lim * planner.sign,
        color="r",
        linestyle="--",
        label=f"a_lim={planner.a_lim:.2f}",
    )
    axs[2].axhline(y=-planner.a_lim * planner.sign, color="r", linestyle="--")
    axs[2].grid(True)
    axs[2].legend(loc="best")

    # ジャーク
    axs[3].plot(t_array, jerk_array, "purple")
    axs[3].set_ylabel("ジャーク (j)")
    axs[3].set_xlabel("時間 (s)")
    axs[3].grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.93])
    plt.show()
